import { DataChannel } from "../DataChannel";

export interface ICustomDataChannelCreator {
  createCustomDataChannel(
    channelName: string,
    rtcConfig?: RTCDataChannelInit
  ): Promise<DataChannel>;
}
