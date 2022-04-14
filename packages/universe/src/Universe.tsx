import React, { useEffect, useRef, useState } from "react";
import useMeasure from "react-use-measure";
import * as THREE from "three";
import styled from "styled-components";

const FullHeightContainer = styled.div`
  width: 100%;
  height: 100vh;
`;

export function Universe() {
  const mountRef = useRef<HTMLDivElement | null>(null);
  const [scene] = useState(new THREE.Scene());
  const [ref, bounds] = useMeasure();
  const [camera] = useState(
    () =>
      new THREE.PerspectiveCamera(75, bounds.width / bounds.height, 0.1, 1000)
  );
  const [renderer] = useState(
    () =>
      new THREE.WebGLRenderer({
        antialias: true,
        alpha: true,
      })
  );

  useEffect(() => {
    const mount = mountRef.current;
    renderer.xr.enabled = true;
    const { devicePixelRatio } = window;
    renderer.setPixelRatio(devicePixelRatio);
    renderer.setSize(bounds.bottom, bounds.height);
    mount?.appendChild(renderer.domElement);

    var geometry = new THREE.BoxGeometry(1, 1, 1);
    var material = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    var cube = new THREE.Mesh(geometry, material);

    scene.add(cube);
    camera.position.z = 5;

    let done = false;
    var animate = function () {
      if (done) {
        return;
      }
      requestAnimationFrame(animate);
      cube.rotation.x += 0.01;
      cube.rotation.y += 0.01;
      renderer.render(scene, camera);
    };

    animate();
    () => {
      done = true;
    };
  }, []);
  useEffect(() => {
    camera.aspect = bounds.width / bounds.height;
    camera.updateProjectionMatrix();
    renderer.setSize(bounds.width, bounds.height);
  }, [bounds]);
  return (
    <FullHeightContainer ref={ref}>
      <div ref={mountRef}></div>
    </FullHeightContainer>
  );
}
