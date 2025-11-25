// camera.js
export class CameraManager {
  constructor(videoEl, cvCanvas) {
    this.video = videoEl;
    this.cvCanvas = cvCanvas;
    this.cvCtx = cvCanvas.getContext('2d');
    this.ready = false;
  }

  async start() {
    // iOS Safari: must be triggered by user gesture, hence Start button
    const constraints = {
      audio: false,
      video: {
        facingMode: 'environment',
        width: { ideal: 640 },
        height: { ideal: 480 }
      }
    };

    const stream = await navigator.mediaDevices.getUserMedia(constraints);
    this.video.srcObject = stream;

    return new Promise(resolve => {
      this.video.onloadedmetadata = () => {
        this.video.play();

        const w = this.video.videoWidth;
        const h = this.video.videoHeight;

        // internal processing size (downscaled for speed)
        this.cvCanvas.width = w / 2;
        this.cvCanvas.height = h / 2;

        this.ready = true;
        resolve({ width: w, height: h });
      };
    });
  }

  grabFrame() {
    if (!this.ready) return null;

    const w = this.cvCanvas.width;
    const h = this.cvCanvas.height;
    this.cvCtx.drawImage(this.video, 0, 0, w, h);
    return this.cvCtx.getImageData(0, 0, w, h);
  }
}
