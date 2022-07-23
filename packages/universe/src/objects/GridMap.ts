import {
  CanvasTexture,
  ClampToEdgeWrapping,
  CustomBlending,
  DoubleSide,
  Group,
  LinearFilter,
  Matrix4,
  Mesh,
  NearestFilter,
  PlaneGeometry,
  ShaderMaterial,
  Texture,
} from "three";
import { defined } from "../../../common/defined";
import { fork } from "../../../common/fork";
import { Color } from "../../../common/Color";
import { transformMatrix } from "../math/transformMatrix";
import { IGridMap } from "../main";

export class GridMap extends Group {
  public onLoad?: () => void;

  private material: ShaderMaterial;

  private plane?: Mesh;

  private current?: IGridMap;

  private next?: IGridMap;

  private loading?: IGridMap;

  constructor() {
    super();

    const mappedColor = defined(
      Color.fromString(
        "#3b4668"
      )
    );
    const occupiedColor = defined(
      Color.fromString(
        "#657197"
      )
    );

    const glColor = (c: Color) =>
      `vec3(${c.r.toFixed(1)}/255.0, ${c.g.toFixed(1)}/255.0, ${c.b.toFixed(
        1
      )}/255.0)`;

    const vertexShader = `
varying vec2 vUv;
void main() {
    vUv = uv;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}`;

    const fragmentShader = `
uniform sampler2D mapTexture;
varying vec2 vUv;

void main() {
    vec4 map = texture2D( mapTexture, vUv.xy );
    vec3 mappedColor = ${glColor(mappedColor)};
    vec3 occupiedColor = ${glColor(occupiedColor)};
    float occupancy = smoothstep(0.5, 0.01, map.r);
    vec3 color = mix(mappedColor, occupiedColor, smoothstep(0.0, 1.0, occupancy));
    gl_FragColor = vec4(color, map.a);
}`;

    this.material = new ShaderMaterial({
      blending: CustomBlending,
      uniforms: {
        mapTexture: { value: new Texture() },
      },
      vertexShader,
      fragmentShader,
      side: DoubleSide,
    });
    this.matrixAutoUpdate = false;
  }

  public set map(map: IGridMap | undefined) {
    fork(this.update(map));
  }

  private async update(map: IGridMap | undefined) {
    if (this.loading) {
      this.next = map;
      return;
    }
    if (this.current === map) {
      return;
    }
    this.loading = map;

    const firstLoad = this.current === undefined;

    if (!map) {
      if (this.plane) {
        this.remove(this.plane);
        this.plane = undefined;
      }
      return;
    }

    if (!this.plane) {
      this.plane = new Mesh(new PlaneGeometry(1, 1), this.material);
      this.plane.position.set(0.5, 0.5, 0);
      this.add(this.plane);
    }

    const { canvas, worldToLocal, origin, width, height, resolution } = map;

    const mapTexture = new CanvasTexture(canvas);

    mapTexture.generateMipmaps = false;
    mapTexture.wrapS = ClampToEdgeWrapping;
    mapTexture.wrapT = ClampToEdgeWrapping;
    mapTexture.minFilter = LinearFilter;
    mapTexture.magFilter = NearestFilter;
    this.material.uniforms.mapTexture.value = mapTexture;

    this.matrix.copy(
      transformMatrix(worldToLocal)
        .multiply(transformMatrix(origin))
        .multiply(
          new Matrix4().makeScale(width * resolution, height * resolution, 1)
        )
    );

    if (firstLoad && this.onLoad) {
      this.onLoad();
    }

    this.current = map;
    this.loading = undefined;
    const { next } = this;
    if (next && next !== map) {
      this.next = undefined;
      await this.update(next);
    }
  }
}
