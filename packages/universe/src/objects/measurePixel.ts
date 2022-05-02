import { PerspectiveCamera, Vector2, Vector3 } from "three";

export function measurePixel(
    position: Vector3,
    camera: PerspectiveCamera,
    resolution: Vector2
) {
    return (
        (position.distanceTo(camera.position) / resolution.x) * camera.aspect
    );
}
