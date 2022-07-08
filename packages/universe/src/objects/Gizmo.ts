import {
  BoxGeometry,
  BufferGeometry,
  Camera,
  CylinderGeometry,
  Float32BufferAttribute,
  Mesh,
  MeshBasicMaterial,
  Object3D,
  Quaternion,
  SphereGeometry,
  TorusGeometry,
  Vector3,
} from "three";

const radialGizmoRadius = 0.3;

export type TranslateGizmoDimension =
  | "X"
  | "Y"
  | "Z"
  | "XYZ"
  | "XY"
  | "YZ"
  | "XZ";
export type TranslatePickerDimension =
  | "X"
  | "Y"
  | "Z"
  | "XYZ"
  | "XY"
  | "YZ"
  | "XZ";
export type TranslateHelperDimension =
  | "START"
  | "END"
  | "DELTA"
  | "X"
  | "Y"
  | "Z";
export type RotateGizmoDimension = "XYZE" | "X" | "Y" | "Z" | "E";
export type RotateHelperDimension = "AXIS";
export type RotatePickerDimension = "XYZE" | "X" | "Y" | "Z" | "E";

const xColor = "#ea719d";

const yColor = "#2ec495";

const zColor = "#18d2ff";

const highlightColor = "#f8f9fc";

const grayColor = "#bac4e2";

export type TransformControlMode = "translate" | "rotate";

export interface ITransformControlsDimensions {
  translation?: TranslateGizmoDimension[];
  rotation?: RotateGizmoDimension[];
}

export class Gizmo extends Object3D {
  readonly isTransformControlsGizmo: boolean = true;

  public gizmo: { [key in TransformControlMode]?: any } = {};

  public picker: { [key in TransformControlMode]?: any } = {};

  public helper: { [key in TransformControlMode]?: any } = {};

  public camera?: Camera;

  public enabled: boolean = true;

  public axis: string | null = null;

  public mode: TransformControlMode = "translate";

  public space: "local" | "world" = "world";

  public size: number = 1;

  public showX: boolean = true;

  public showY: boolean = true;

  public showZ: boolean = true;

  public worldPosition: Vector3 = new Vector3();

  public worldPositionStart: Vector3 = new Vector3();

  public worldQuaternion: Quaternion = new Quaternion();

  public worldQuaternionStart: Quaternion = new Quaternion();

  public cameraPosition: Vector3 = new Vector3();

  public cameraQuaternion: Quaternion = new Quaternion();

  public rotationAxis: Vector3 = new Vector3();

  public eye: Vector3 = new Vector3();

  constructor(
    dimensions: ITransformControlsDimensions = {
      translation: ["X", "Y", "Z", "XY", "XYZ", "XY", "XZ", "YZ"],
      rotation: ["X", "Y", "Z"],
    }
  ) {
    super();
    const baseball = 0.076;
    // add a sphere for raycast
    const sphere = new Mesh(
      new SphereGeometry(0.75 / 2, 16, 8),
      new MeshBasicMaterial({
        color: 0xffffff,
        opacity: 0,
        transparent: true,
      })
    );
    this.add(sphere);
    const size = 0.75;
    this.scale.set(
      (1 / size) * baseball,
      (1 / size) * baseball,
      (1 / size) * baseball
    );
    this.type = "TransformControlsGizmo";

    const gizmoMaterial = new MeshBasicMaterial({
      depthTest: false,
      depthWrite: false,
      fog: false,
      toneMapped: false,
      transparent: true,
    });

    // Make unique material for each axis/color

    const matInvisible = gizmoMaterial.clone() as MeshBasicMaterial;
    matInvisible.opacity = 0.15;

    const matRed = gizmoMaterial.clone() as MeshBasicMaterial;
    matRed.color.setStyle(xColor);

    const matGreen = gizmoMaterial.clone() as MeshBasicMaterial;
    matGreen.color.setStyle(yColor);

    const matBlue = gizmoMaterial.clone() as MeshBasicMaterial;
    matBlue.color.setStyle(zColor);

    const matRedTransparent = gizmoMaterial.clone() as MeshBasicMaterial;
    matRedTransparent.color.setStyle(xColor);
    matRedTransparent.opacity = 0.5;

    const matGreenTransparent = gizmoMaterial.clone() as MeshBasicMaterial;
    matGreenTransparent.color.setStyle(yColor);
    matGreenTransparent.opacity = 0.5;

    const matBlueTransparent = gizmoMaterial.clone() as MeshBasicMaterial;
    matBlueTransparent.color.setStyle(zColor);
    matBlueTransparent.opacity = 0.5;

    const matHighlightTransparent = gizmoMaterial.clone() as MeshBasicMaterial;
    matHighlightTransparent.color.setStyle(highlightColor);
    matHighlightTransparent.opacity = 0.25;

    const matYellow = gizmoMaterial.clone() as MeshBasicMaterial;
    matYellow.color.setStyle(highlightColor);

    const matGray = gizmoMaterial.clone() as MeshBasicMaterial;
    matGray.color.setStyle(grayColor);

    // reusable geometry

    const arrowGeometry = new CylinderGeometry(0, 0.03, 0.06, 12);
    arrowGeometry.translate(0, 0.05, 0);

    const scaleHandleGeometry = new BoxGeometry(0.08, 0.08, 0.08);
    scaleHandleGeometry.translate(0, 0.04, 0);

    const lineGeometry = new BufferGeometry();
    lineGeometry.setAttribute(
      "position",
      new Float32BufferAttribute([0, 0, 0, 1, 0, 0], 3)
    );

    const lineGeometry2 = new CylinderGeometry(0.0075, 0.0075, 0.1, 3);
    lineGeometry2.translate(0, 0.27, 0);

    function CircleGeometry(
      radius: number = radialGizmoRadius,
      arc: number = 0.25
    ) {
      const geometry = new TorusGeometry(
        radius,
        0.0075,
        3,
        64,
        arc * Math.PI * 2
      );
      geometry.rotateY(Math.PI / 2);
      geometry.rotateX(Math.PI / 2);
      return geometry;
    }

    // Gizmo definitions - custom hierarchy definitions for setupGizmo() function
    const gizmoTranslate: { [key in TranslateGizmoDimension]: any[] } = {
      X: [
        [new Mesh(arrowGeometry, matRed), [0.3, 0, 0], [0, 0, -Math.PI / 2]],
        [new Mesh(lineGeometry2, matRed), [0, 0, 0], [0, 0, -Math.PI / 2]],
      ],
      Y: [
        [new Mesh(arrowGeometry, matGreen), [0, 0.3, 0]],
        [new Mesh(lineGeometry2, matGreen)],
      ],
      Z: [
        [new Mesh(arrowGeometry, matBlue), [0, 0, 0.3], [Math.PI / 2, 0, 0]],
        [new Mesh(lineGeometry2, matBlue), null, [Math.PI / 2, 0, 0]],
      ],
      XYZ: [],
      XY: [
        [
          new Mesh(new BoxGeometry(0.1, 0.1, 0.01), matBlueTransparent.clone()),
          [0.15, 0.15, 0],
        ],
      ],
      YZ: [
        [
          new Mesh(new BoxGeometry(0.1, 0.1, 0.01), matRedTransparent.clone()),
          [0, 0.15, 0.15],
          [0, Math.PI / 2, 0],
        ],
      ],
      XZ: [
        [
          new Mesh(
            new BoxGeometry(0.1, 0.1, 0.01),
            matGreenTransparent.clone()
          ),
          [0.15, 0, 0.15],
          [-Math.PI / 2, 0, 0],
        ],
      ],
    };

    const helperTranslate: { [key in TranslateHelperDimension]: any } = {
      START: [],
      END: [],
      DELTA: [],
      X: [],
      Y: [],
      Z: [],
    };

    const gizmoRotate: { [key in RotateGizmoDimension]: any } = {
      XYZE: [
        [new Mesh(CircleGeometry(0.5, 1), matGray), null, [0, Math.PI / 2, 0]],
      ],
      X: [[new Mesh(CircleGeometry(), matRed), null, [Math.PI, 0, 0]]],
      Y: [
        [
          new Mesh(CircleGeometry(), matGreen),
          null,
          [0, -Math.PI / 2, Math.PI / 2],
        ],
      ],
      Z: [
        [
          new Mesh(CircleGeometry(), matBlue),
          null,
          [-Math.PI, Math.PI / -2, 0],
        ],
      ],
      E: [
        [
          new Mesh(CircleGeometry(0.75, 1), matHighlightTransparent),
          null,
          [0, Math.PI / 2, 0],
        ],
      ],
    };

    const helperRotate: { [key in RotateHelperDimension]: any } = {
      AXIS: [],
    };

    // Creates an Object3D with gizmos described in custom hierarchy definition.

    function setupGizmo(
      gizmoMap: any,
      mode: TransformControlMode,
      gizmoDimensions?: string[]
    ) {
      const gizmo = new Object3D();

      // eslint-disable-next-line no-restricted-syntax
      for (const name in gizmoMap) {
        if (gizmoDimensions !== undefined && !gizmoDimensions.includes(name)) {
          // eslint-disable-next-line no-continue
          continue;
        }

        // eslint-disable-next-line no-plusplus
        for (let i = gizmoMap[name].length; i--; ) {
          const object = gizmoMap[name][i][0].clone();
          const position = gizmoMap[name][i][1];
          const rotation = gizmoMap[name][i][2];
          const scale = gizmoMap[name][i][3];
          const tag = gizmoMap[name][i][4];

          // name and tag properties are essential for picking and updating logic.
          object.name = name;
          object.tag = tag;
          object.mode = mode;

          if (position) {
            object.position.set(position[0], position[1], position[2]);
          }

          if (rotation) {
            object.rotation.set(rotation[0], rotation[1], rotation[2]);
          }

          if (scale) {
            object.scale.set(scale[0], scale[1], scale[2]);
          }

          object.updateMatrix();

          const tempGeometry = object.geometry.clone();
          tempGeometry.applyMatrix4(object.matrix);
          object.geometry = tempGeometry;
          object.renderOrder = Infinity;

          object.position.set(0, 0, 0);
          object.rotation.set(0, 0, 0);
          object.scale.set(1, 1, 1);

          gizmo.add(object);
        }
      }

      return gizmo;
    }

    // Gizmo creation

    this.gizmo = {};
    this.picker = {};
    this.helper = {};

    this.add(
      (this.gizmo.translate = setupGizmo(
        gizmoTranslate,
        "translate",
        dimensions.translation
      ))
    );
    this.add(
      (this.gizmo.rotate = setupGizmo(
        gizmoRotate,
        "rotate",
        dimensions.rotation
      ))
    );

    this.add(
      (this.helper.translate = setupGizmo(helperTranslate, "translate"))
    );
    this.add((this.helper.rotate = setupGizmo(helperRotate, "rotate")));
  }
}
