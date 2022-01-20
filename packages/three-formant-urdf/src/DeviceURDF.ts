import { Device, Fleet, IJointState } from "@formant/data-sdk";
import { defined } from "../../common/defined";
import * as THREE from "three";
import { LoadingManager, Mesh, Scene } from "three";
import { ColladaLoader } from "three/examples/jsm/loaders/ColladaLoader.js";
import URDFLoader, { URDFRobot } from "urdf-loader";
import { Object3D } from "three";
import * as JSZip from "jszip";

export class DeviceURDF extends THREE.Object3D {
  private robot?: URDFRobot;
  private meshs: Mesh[] = [];

  private opacity = 1.0;
  private transparent = false;
  private color = new THREE.Color("white");

  constructor(private device: Device) {
    super();
    return this
  }

  async setup(): Promise<any>  {
    try{
      const device = defined(this.device);
      const config = await device.getConfiguration();
      if (!config.urdfFiles || config.urdfFiles.length === 0) {
        throw new Error("No URDF files found for device " + device.name);
      }
      const zipFileUrl = await Fleet.getFileUrl(config.urdfFiles[0]);
      let blobUrl = await this.loadURDFIntoBlob(zipFileUrl);
      if (blobUrl) {
        const manager = new LoadingManager();
        const loader = new URDFLoader(manager);
  
        loader.loadMeshCb = ((
          path: string,
          loadingManager: LoadingManager,
          done: (s: Scene) => void
        ) => {
          const daeLoader = new ColladaLoader(loadingManager);
          daeLoader.load(path, (dae) => {
            const { opacity, transparent, color } = this;
            dae.scene.children = dae.scene.children.filter(
              (_) => !_.type.endsWith("Light")
            );
            dae.scene.traverse((_) => {
              if (_ instanceof Mesh) {
                this.meshs.push(_);
                _.geometry.computeVertexNormals();
  
                _.material = new THREE.MeshPhongMaterial({
                  color,
                  opacity,
                  transparent,
                });
              }
            });
  
            done(dae.scene);
          });
        }).bind(loader);
  
        loader.load(blobUrl, async (robot) => {
          this.robot = robot;
          this.add(robot as unknown as Object3D);
          this.startListening();
        });

        return Promise.resolve()
      } else {
        throw new Error("URDF file zip was invalid for device " + device.name);
      }
    } catch(e: any){
      console.log(e.message)
      return Promise.reject(e)
    }
    
  }

  loadURDFIntoBlob = async (zipPath: string): Promise<string | false> => {
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
  };

  async startListening() {
    const device = defined(this.device);
    await device.startRealtimeConnection();
    const manipulators = await device.getRealtimeManipulators();
    if (manipulators.length > 0) {
      const manipulator = manipulators[0];
      await manipulator.synchronize();
      await manipulator.addCurrentJointStateListener(
        (jointState: IJointState) => {
          const robot = defined(this.robot);

          const { name: names } = jointState;

          Object.keys(robot.joints).forEach((_) => {
            const joint = robot.joints[_];

            const index = names.indexOf(_);

            const effort = jointState.effort?.[index] ?? 0;
            const position = jointState.position?.[index] ?? 0;
            const velocity = jointState.velocity?.[index] ?? 0;

            joint.setJointValue(position, effort, velocity);
          });
        }
      );
    }
  }
}

export async function getDeviceURDF (device: Device) {  return await new DeviceURDF(device).setup() }
