import * as THREE from "three";
import { DeviceURDF } from "@formant/three-formant-urdf";
import { Object3D } from "three";
import { Fleet, Authentication } from "@formant/data-sdk";

const scene = new THREE.Scene();

// Add device URDF using the current device context
Authentication.waitTilAuthenticated().then(async () => {
  const urdf = new DeviceURDF(await Fleet.getCurrentDevice());
  urdf.scale.set(4, 4, 4);
  urdf.rotateX(-Math.PI / 2);
  urdf.rotateZ(-Math.PI / 4);
  scene.add(urdf as Object3D);
  await urdf.setup();
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
