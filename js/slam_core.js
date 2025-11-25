// slam_core.js
export class SlamCore {
  constructor() {
    this.initialized = false;

    this.prevGray = null;
    this.prevPoints = null;   // 2D points in prev frame
    this.pose = {
      R: cv.Mat.eye(3, 3, cv.CV_64F),
      t: cv.Mat.zeros(3, 1, cv.CV_64F)
    };

    this.mapPoints = []; // array of { X: [x,y,z] }
  }

  _ensureMats(width, height) {
    if (!this.gray) {
      this.gray = new cv.Mat(height, width, cv.CV_8UC1);
      this.currGray = new cv.Mat(height, width, cv.CV_8UC1);
    }
  }

  /**
   * Process a new frame:
   *   - imageData: ImageData from canvas (RGBA)
   * Returns:
   *   - current pose & sparse point cloud (subset)
   */
  processFrame(imageData) {
    const width = imageData.width;
    const height = imageData.height;

    this._ensureMats(width, height);

    // Convert RGBA → Gray
    const rgbaMat = cv.matFromImageData(imageData);
    cv.cvtColor(rgbaMat, this.currGray, cv.COLOR_RGBA2GRAY);
    rgbaMat.delete();

    if (!this.initialized) {
      this.prevGray = this.currGray.clone();
      this.prevPoints = this._detectFeatures(this.prevGray);
      this.initialized = true;

      return {
        pose: this.pose,
        mapPoints: this.mapPoints
      };
    }

    // Track prevPoints → currPoints
    const { currPoints, status } = this._trackFeatures(
      this.prevGray,
      this.currGray,
      this.prevPoints
    );

    // Filter good points
    const goodPrev = [];
    const goodCurr = [];
    for (let i = 0; i < status.rows; i++) {
      if (status.data[i] === 1) {
        goodPrev.push(this.prevPoints.data32F[2 * i], this.prevPoints.data32F[2 * i + 1]);
        goodCurr.push(currPoints.data32F[2 * i], currPoints.data32F[2 * i + 1]);
      }
    }

    if (goodPrev.length < 16) {
      // Not enough points, re-detect
      this.prevGray = this.currGray.clone();
      this.prevPoints = this._detectFeatures(this.prevGray);
      return {
        pose: this.pose,
        mapPoints: this.mapPoints
      };
    }

    const prevMat = cv.matFromArray(goodPrev.length / 2, 1, cv.CV_32FC2, goodPrev);
    const currMat = cv.matFromArray(goodCurr.length / 2, 1, cv.CV_32FC2, goodCurr);

    // Assume some camera intrinsics (approx phone camera)
    const fx = 600, fy = 600, cx = width / 2, cy = height / 2;
    const K = cv.matFromArray(3, 3, cv.CV_64F,
      [fx, 0, cx,
       0, fy, cy,
       0, 0, 1]);

    // Essential matrix
    const mask = new cv.Mat();
    const E = cv.findEssentialMat(currMat, prevMat, K, cv.RANSAC, 0.999, 1.0, mask);

    const R = new cv.Mat();
    const t = new cv.Mat();
    const points = new cv.Mat();

    cv.recoverPose(E, currMat, prevMat, K, R, t, mask);

    // Compose global pose: pose_new = pose_old * (R, t)
    // For SLAM-lite we approximate and just accumulate:
    this.pose.R = R.mul(this.pose.R); // rough, we can refine later
    this.pose.t = this._addTrans(this.pose.t, t);

    // Triangulate points (local)
    const P0 = cv.Mat.zeros(3, 4, cv.CV_64F);
    for (let i = 0; i < 3; i++) {
      P0.doublePtr(i, i)[0] = 1.0;
    }

    const P1 = new cv.Mat(3, 4, cv.CV_64F);
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        P1.doublePtr(r, c)[0] = R.doublePtr(r, c)[0];
      }
      P1.doublePtr(r, 3)[0] = t.doublePtr(r, 0)[0];
    }

    cv.triangulatePoints(P0, P1, prevMat, currMat, points);

    // Convert homogeneous → 3D and push to map
    for (let i = 0; i < points.cols; i++) {
      const w = points.data64F[4 * i + 3];
      if (Math.abs(w) < 1e-6) continue;

      const X = points.data64F[4 * i] / w;
      const Y = points.data64F[4 * i + 1] / w;
      const Z = points.data64F[4 * i + 2] / w;

      this.mapPoints.push({ X, Y, Z });
    }

    // Prepare for next frame
    this.prevGray = this.currGray.clone();
    this.prevPoints = currMat.clone();

    E.delete();
    R.delete();
    t.delete();
    P0.delete();
    P1.delete();
    mask.delete();
    points.delete();
    prevMat.delete();
    // currMat kept as prevPoints reference

    return {
      pose: this.pose,
      mapPoints: this.mapPoints
    };
  }

  _addTrans(t1, t2) {
    const out = new cv.Mat(3, 1, cv.CV_64F);
    for (let i = 0; i < 3; i++) {
      out.doublePtr(i, 0)[0] =
        t1.doublePtr(i, 0)[0] + t2.doublePtr(i, 0)[0];
    }
    return out;
  }

  _detectFeatures(gray) {
    const corners = new cv.Mat();
    const maxCorners = 300;
    const qualityLevel = 0.01;
    const minDistance = 10;
    const blockSize = 3;
    const useHarrisDetector = false;
    const k = 0.04;

    cv.goodFeaturesToTrack(
      gray,
      corners,
      maxCorners,
      qualityLevel,
      minDistance,
      new cv.Mat(),
      blockSize,
      useHarrisDetector,
      k
    );

    return corners;
  }

  _trackFeatures(prevGray, currGray, prevPoints) {
    const currPoints = new cv.Mat();
    const status = new cv.Mat();
    const err = new cv.Mat();

    const winSize = new cv.Size(21, 21);
    const maxLevel = 3;
    const criteria = new cv.TermCriteria(
      cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT,
      30,
      0.01
    );

    cv.calcOpticalFlowPyrLK(
      prevGray,
      currGray,
      prevPoints,
      currPoints,
      status,
      err,
      winSize,
      maxLevel,
      criteria
    );

    err.delete();
    return { currPoints, status };
  }
}
