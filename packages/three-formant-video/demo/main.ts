import * as THREE from "three";
import { Authentication, Fleet } from "@formant/data-sdk";
import { DeviceVideo } from "../src/DeviceVideo";

await Authentication.waitTilAuthenticated();
const device = await Fleet.getCurrentDevice();
await device.startRealtimeConnection();

const scene = new THREE.Scene();

const videoStreams = await device.getRealtimeVideoStreams();

// Add device URDF using the current device context
videoStreams.forEach((_, i) => {
  const v = new DeviceVideo(device, _.name);
  v.scale.set(2, 2, 2);
  v.position.setX(i * 3);
  v.rotateX(90);
  scene.add(v);
});

const camera = new THREE.PerspectiveCamera(
  75,
  window.innerWidth / window.innerHeight,
  0.1,
  1000
);

camera.position.z = 5;

/// BOILERPLATE BELOW

const directionalLight = new THREE.DirectionalLight(0xffffff, 1.0);
directionalLight.castShadow = true;
directionalLight.shadow.mapSize.setScalar(1024);
directionalLight.position.set(5, 30, 5);
scene.add(directionalLight);

const ambientLight = new THREE.AmbientLight(0xffffff, 0.2);
scene.add(ambientLight);

const renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const animate = function () {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
};

animate();

window.addEventListener("resize", onWindowResize, false);

function onWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();

  renderer.setSize(window.innerWidth, window.innerHeight);
}
