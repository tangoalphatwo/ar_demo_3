// main.js
import { CameraManager } from './camera.js';
import { ARRenderer } from './renderer.js';
import { SlamCore } from './slam_core.js';
import { PlaneEstimator } from './plane_estimator.js';

window.addEventListener('load', () => {
  const videoEl = document.getElementById('camera');
  const cvCanvas = document.getElementById('cvCanvas');
  const threeCanvas = document.getElementById('threeCanvas');
  const startBtn = document.getElementById('startBtn');
  const statusEl = document.getElementById('status');

  const camera = new CameraManager(videoEl, cvCanvas);
  const renderer = new ARRenderer(threeCanvas);
  const slam = new SlamCore();
  const planeEstimator = new PlaneEstimator();

  let running = false;

  startBtn.addEventListener('click', async () => {
    try {
      statusEl.textContent = 'Starting camera...';
      await camera.start();

      renderer.setVideoTexture(videoEl);
      statusEl.textContent = 'Initializing OpenCV...';

      // Wait for OpenCV to be fully ready
      cv['onRuntimeInitialized'] = () => {
        statusEl.textContent = 'Running SLAM-lite...';
        running = true;
        loop();
      };
    } catch (err) {
      console.error(err);
      statusEl.textContent = 'Error starting camera';
    }
  });

  function loop() {
    if (!running) return;

    const frame = camera.grabFrame();
    if (frame) {
      const { pose, mapPoints } = slam.processFrame(frame);

      const plane = planeEstimator.estimatePlane(mapPoints);
      const planePose = planeEstimator.planeToPose(plane);

      renderer.updatePlanePose(planePose);
      renderer.render();
    }

    requestAnimationFrame(loop);
  }

  window.addEventListener('resize', () => renderer.resize());
});
