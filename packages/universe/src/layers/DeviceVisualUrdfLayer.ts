import * as JSZip from "jszip";
import * as uuid from "uuid";
import { Object3D, Event, Group, Material, Mesh, BufferGeometry } from "three";
import { defined } from "../../../common/defined";
import { IJointState } from "../../../data-sdk/src/model/IJointState";
import { Urdf } from "../objects/Urdf";
import { IUniverseData } from "../model/IUniverseData";
import { LayerSuggestion } from "./LayerRegistry";
import { UniverseLayer } from "./UniverseLayer";

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

export class DeviceVisualUrdfLayer extends UniverseLayer {
  static layerTypeId: string = "device_visual_urdf";

  static commonName = "URDF";

  static description = "A 3D model to represent a robot.";

  static usesData = true;

  static fields = {
    ghosted: {
      name: "Ghosted",
      description: "If you would like this URDF to appear ghosted transparent.",
      placeholder: "false",
      value: "",
      type: "text",
      location: ["create"],
    },
  };

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
              layerType: DeviceVisualUrdfLayer.layerTypeId,
            });
          }
        }
      );
    }
    return suggestions;
  }

  urdf: Urdf | undefined;

  loaded: boolean = false;

  init() {
    const ninetyDegrees = Math.PI / 2;
    this.rotation.set(-ninetyDegrees, 0, 0);
    const dataSource = defined(this.layerDataSources)[0];
    if (
      dataSource &&
      dataSource.sourceType === "realtime" &&
      this.layerContext
    ) {
      defined(this.universeData).subscribeToJointState(
        this.layerContext,
        defined(dataSource),
        this.onData
      );
      defined(this.universeData)
        .getUrdfs(defined(this.layerContext))
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
    this.urdf = new Urdf(
      blobUrl,
      { ghosted: this.layerFields?.ghosted?.value === "true" },
      this.onLoad
    );
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

  override onLayerPartsRequested(): {
    [x: string]:
      | Object3D<Event>
      | Material
      | Mesh<BufferGeometry, Material | Material[]>
      | Group
      | Urdf
      | undefined;
  } {
    return {
      urdf: this.urdf,
    };
  }
}
