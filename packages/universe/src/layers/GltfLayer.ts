import { IUniverseData, UniverseDataSource } from "../model/IUniverseData";
import { GLTFLoader } from "../../three-utils/GLTFLoader";
import { TransformLayer } from "./TransformLayer";
import { UniverseLayerContent } from "./UniverseLayerContent";
import { LayerFields, LayerField } from "../model/LayerField";
import { defined } from "../../../common/defined";

export class GltfLayer extends UniverseLayerContent {
  static id = "3dmodel";

  static commonName = "3D Model";

  static description = "A 3D model.";

  static usesData = false;

  static fields = {
    url: {
      name: "URL",
      description: "The URL of the 3D model (*.gltf only)",
      placeholder: "https://example.com/model.gltf",
      value: "",
      type: "text",
      location: ["create"],
    },
  };

  static createDefault(
    layerId: string,
    _universeData: IUniverseData,
    deviceId: string,
    _universeDataSources?: UniverseDataSource[],
    fields?: LayerFields
  ): TransformLayer<GltfLayer> {
    return new TransformLayer(
      layerId,
      new GltfLayer(layerId, (fields || {}).url),
      deviceId
    );
  }

  constructor(layerId?: string, urlField?: LayerField) {
    super(defined(layerId));
    if (urlField && urlField.type === "text" && urlField.value) {
      const loader = new GLTFLoader();
      loader.load(urlField.value, (gltf) => {
        const ninetyDegrees = Math.PI / 2;
        gltf.scene.rotation.set(ninetyDegrees, 0, 0);
        this.add(gltf.scene);
      });
    }
  }
}
