import { PerspectiveCamera } from "three";
import * as JSZip from "jszip";
import * as uuid from "uuid";
import { defined } from "../../../common/defined";
import { IJointState } from "../../../data-sdk/src/model/IJointState";
import { Urdf } from "../objects/Urdf";
import { IUniverseData, UniverseDataSource } from "../model/IUniverseData";
import { LayerSuggestion } from "./LayerRegistry";
import { TransformLayer } from "./TransformLayer";
import { UniverseLayerContent } from "./UniverseLayerContent";
import { LayerFields } from "../model/LayerField";

async function loadURDFIntoBlob(zipPath: string): Promise<string | false> {
  const data = await fetch(zipPath).then((_) => _.arrayBuffer());
  const zipFile = await JSZip.loadAsync(data);
  // find a root urdf file
  const urdfRoot = Object.keys(zipFile.files).find((_) =>
    _.toLowerCase().endsWith("urdf")
  );
  if (urdfRoot) {
    // load the urdf as a string
    let urdf = await zipFile.files[urdfRoot].async("string");

    // lets get all the png images and make blob urls for them
    const images = Object.keys(zipFile.files).filter(
      (_) => _.endsWith("png") && _ !== urdfRoot
    );
    // create a map of the images to their urls
    const imageUrls: { [key in string]: string } = {};
    await Promise.all(
      images.map(async (f) => {
        const file = zipFile.files[f];
        const txt = await zipFile.files[file.name].async("arraybuffer");
        imageUrls[f] = URL.createObjectURL(
          new Blob([txt], {
            type: "image/png",
          })
        );
      })
    );

    // for all other files ( should just be models )
    const nonImages = Object.keys(zipFile.files).filter(
      (_) => !_.endsWith(".png") && _ !== urdfRoot
    );
    await Promise.all(
      nonImages.map(async (f) => {
        const file = zipFile.files[f];
        if (!file.dir) {
          let txt = await zipFile.files[file.name].async("string");

          // replace all image references ( non-relative only )
          images.forEach((imageKey) => {
            const keys = imageKey.split("/");
            const key = keys[keys.length - 1];
            txt = txt.replace(
              new RegExp(key, "g"),
              imageUrls[imageKey].replace(`blob:${window.location.origin}/`, "")
            );
          });
          const modelUrl = URL.createObjectURL(
            new Blob([txt], {
              type: "text/plain",
            })
          ).replace(`blob:${window.location.origin}/`, "");
          // replace the reference to the model in the root urdf
          urdf = urdf.replace(new RegExp(`package://${f}`, "g"), modelUrl);

          urdf = urdf.replace(new RegExp(f, "g"), modelUrl);
        }
      })
    );
    // create the urdf blob url
    return URL.createObjectURL(
      new Blob([urdf], {
        type: "text/plain",
      })
    );
  }
  return false;
}

export class DeviceVisualUrdfLayer extends UniverseLayerContent {
  static id = "device_visual_urdf";

  static commonName = "URDF";

  static description = "A 3D model to represent a robot.";

  static usesData = true;

  static createDefault(
    universeData: IUniverseData,
    deviceId: string,
    universeDataSources?: UniverseDataSource[],
    _fields?: LayerFields,
    _getCurrentCamera?: () => PerspectiveCamera
  ): TransformLayer<DeviceVisualUrdfLayer> {
    return new TransformLayer(
      new DeviceVisualUrdfLayer(
        universeData,
        defined(universeDataSources)[0],
        deviceId
      ),
      deviceId
    );
  }

  static async getLayerSuggestions(
    universeData: IUniverseData,
    deviceContext?: string
  ): Promise<LayerSuggestion[]> {
    const suggestions: LayerSuggestion[] = [];

    if (deviceContext) {
      (await universeData.getTeleopRosStreams(deviceContext)).forEach(
        (stream) => {
          if (stream.topicType === "sensor_msgs/JointState") {
            suggestions.push({
              sources: [
                {
                  id: uuid.v4(),
                  sourceType: "realtime",
                  rosTopicName: stream.topicName,
                  rosTopicType: stream.topicType,
                },
              ],
              layerType: DeviceVisualUrdfLayer.id,
            });
          }
        }
      );
    }
    return suggestions;
  }

  urdf: Urdf | undefined;

  loaded: boolean = false;

  constructor(
    private universeData?: IUniverseData,
    private dataSource?: UniverseDataSource,
    private deviceId?: string
  ) {
    super();
    if (dataSource && dataSource.sourceType === "realtime" && deviceId) {
      defined(this.universeData).subscribeToJointState(
        deviceId,
        defined(this.dataSource),
        this.onData
      );
      defined(universeData)
        .getUrdfs(defined(this.deviceId))
        .then((_) => {
          if (_.length === 0) {
            return;
          }
          loadURDFIntoBlob(_[0])
            .then((blobUrl) => {
              if (blobUrl !== false) {
                this.loadAllUrdfs(blobUrl);
              }
            })
            .catch((e) => {
              throw e;
            });
        })
        .catch((e) => {
          throw e;
        });
    }
  }

  loadAllUrdfs = (blobUrl: string) => {
    this.urdf = new Urdf(blobUrl, undefined, this.onLoad);
    this.add(this.urdf);
  };

  onLoad = () => {
    this.loaded = true;
  };

  onData = (data: IJointState) => {
    if (this.urdf && this.loaded) {
      this.urdf.jointState = data;
    }
  };
}
