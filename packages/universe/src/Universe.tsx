import React, { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import styled from "styled-components";
import { Measure } from "@formant/ui-sdk";

const MeasureContainer = styled.div`
  width: 100%;
  height: 100vh;

  > div {
    overflow: hidden;
    width: 100%;
    height: 100%;
  }
`;

export function Universe() {
  const mountRef = useRef<HTMLDivElement | null>(null);
  const [bounds, setBounds] = useState({ width: 1, height: 1 });
  const [scene] = useState(new THREE.Scene());
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

  useEffect(function () {
    const mount = mountRef.current;
    renderer.xr.enabled = true;
    const { devicePixelRatio } = window;
    renderer.setPixelRatio(devicePixelRatio);
    renderer.setSize(bounds.width, bounds.height);
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

  useEffect(
    function () {
      camera.aspect = bounds.width / bounds.height;
      camera.updateProjectionMatrix();
      renderer.setSize(bounds.width, bounds.height);
    },
    [bounds]
  );
  const onResize = (width: number, height: number) => {
    setBounds({ width, height });
  };

  return (
    <MeasureContainer>
      <Measure onResize={onResize}>
        <div ref={mountRef}></div>
      </Measure>
    </MeasureContainer>
  );
}
