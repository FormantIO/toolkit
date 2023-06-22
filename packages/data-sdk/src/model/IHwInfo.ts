import { IAudioDevice } from "./IAudioInfo";
import { IKernelInfo } from "./IKernelInfo";
import { INetworkInfo } from "./InetworkInfo";
import { INodeInfo } from "./INodeInfo";
import { IOnvifDevice } from "./IOnvifDevice";
import { IOsInfo } from "./IOsInfo";
import { IVideoDevice } from "./IVideoDevice";

export interface IHwInfo {
  hwEncodingAvailable?: boolean;
  nodeInfo?: INodeInfo;
  kernelInfo?: IKernelInfo;
  osInfo?: IOsInfo;
  networkInfo?: INetworkInfo;
  videoDevices?: IVideoDevice[];
  audioCaptureDevices?: IAudioDevice[];
  onvifDevices?: IOnvifDevice[];
}
