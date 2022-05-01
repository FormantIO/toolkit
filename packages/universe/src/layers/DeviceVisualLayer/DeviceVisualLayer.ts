import { PerspectiveCamera, Vector2 } from "three";
import * as JSZip from "jszip";
import * as uuid from "uuid";
import { defined } from "../../../../common/defined";
import { IJointState } from "../../../../data-sdk/src/model/IJointState";
import { ITransformNode } from "../../../../data-sdk/src/model/ITransformNode";
import { TransformTree } from "./TransformTree";
import { Urdf } from "./Urdf";
import { ITransformTreeNode } from "./transformTreeLoader";
import { IUniverseData, UniverseDataSource } from "../../IUniverseData";
import { LayerSuggestion } from "../LayerRegistry";
import { TransformLayer } from "../TransformLayer";
import { LayerFields, UniverseLayerContent } from "../UniverseLayerContent";

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
    for (const f of images) {
      const file = zipFile.files[f];
      const txt = await zipFile.files[file.name].async("arraybuffer");
      imageUrls[f] = URL.createObjectURL(
        new Blob([txt], {
          type: "image/png",
        })
      );
    }

    // for all other files ( should just be models )
    const nonImages = Object.keys(zipFile.files).filter(
      (_) => !_.endsWith(".png") && _ !== urdfRoot
    );
    for (const f of nonImages) {
      const file = zipFile.files[f];
      if (!file.dir) {
        let txt = await zipFile.files[file.name].async("string");

        // replace all image references ( non-relative only )
        for (const imageKey of images) {
          const keys = imageKey.split("/");
          const key = keys[keys.length - 1];
          txt = txt.replace(
            new RegExp(key, "g"),
            imageUrls[imageKey].replace(`blob:${location.origin}/`, "")
          );
        }
        const modelUrl = URL.createObjectURL(
          new Blob([txt], {
            type: "text/plain",
          })
        ).replace(`blob:${location.origin}/`, "");
        // replace the reference to the model in the root urdf
        urdf = urdf.replace(new RegExp("package://" + f, "g"), modelUrl);

        urdf = urdf.replace(new RegExp(f, "g"), modelUrl);
      }
    }
    // create the urdf blob url
    return URL.createObjectURL(
      new Blob([urdf], {
        type: "text/plain",
      })
    );
  }
  return false;
}

export class DeviceVisualLayer extends UniverseLayerContent {
  static id = "device_visual";

  static commonName = "Device Visual";

  static description = "A 3D model to represent a robot.";

  static usesData = true;

  static createDefault(
    universeData: IUniverseData,
    deviceId: string,
    universeDataSources?: UniverseDataSource[],
    _fields?: LayerFields,
    getCurrentCamera?: () => PerspectiveCamera
  ): TransformLayer<DeviceVisualLayer> {
    return new TransformLayer(
      new DeviceVisualLayer(
        universeData,
        defined(universeDataSources)[0],
        deviceId,
        getCurrentCamera
      )
    );
  }

  static getLayerSuggestions(
    universeData: IUniverseData,
    deviceContext?: string
  ): LayerSuggestion[] {
    const suggestions: LayerSuggestion[] = [];

    if (deviceContext) {
      universeData.getTeleopRosStreams().forEach((stream) => {
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
            layerType: DeviceVisualLayer.id,
          });
        }
      });

      universeData.getTelemetryStreams().forEach((stream) => {
        if (
          universeData.getTelemetryStreamType(deviceContext, stream.name) ===
          "transform tree"
        ) {
          suggestions.push({
            sources: [
              {
                id: uuid.v4(),
                sourceType: "telemetry",
                streamName: stream.name,
                streamType: "transform tree",
              },
            ],
            layerType: DeviceVisualLayer.id,
          });
        }
      });
    }
    return suggestions;
  }

  urdf: Urdf | undefined;

  transformTree: TransformTree | undefined;

  loaded: boolean = false;

  constructor(
    private universeData?: IUniverseData,
    private dataSource?: UniverseDataSource,
    private deviceId?: string,
    getCamera?: () => PerspectiveCamera
  ) {
    super();
    if (dataSource && dataSource.sourceType === "realtime") {
      defined(this.universeData).subscribeToDataSource(
        defined(this.dataSource),
        this.onData
      );
      defined(universeData)
        .getUrdfs(defined(this.deviceId))
        .then((_) => {
          if (_ === false) {
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
    } else if (getCamera) {
      this.transformTree = new TransformTree(getCamera());
      this.add(this.transformTree);
      defined(this.universeData).subscribeToDataSource(
        defined(dataSource),
        this.onTransformTreeData
      );
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

  onTransformTreeData = async (data: ITransformNode) => {
    let transformData: ITransformTreeNode | undefined;
    if (data.url) {
      const result = await fetch(data.url);
      transformData = (await result.json()) as ITransformTreeNode;
    }
    if (transformData && this.transformTree) {
      this.transformTree.nodes = [transformData];
      this.transformTree.resolution = new Vector2(600, 400);
      this.transformTree.update();
    }
  };
}
