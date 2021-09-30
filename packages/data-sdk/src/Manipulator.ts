import { Device, RealtimeDataStream } from "./Device";

export type RealtimeManipulatorConfig = {
  currentJointStateStream: RealtimeDataStream;
  plannedJointStateStream?: RealtimeDataStream;
  planValidStream?: RealtimeDataStream;
  endEffectorStream?: RealtimeDataStream;
  endEffectorLinkName?: string;
  baseReferenceFrame?: string;
  localFrame?: string;
};

export class Manipulator {
  constructor(
    private device: Device,
    private config: RealtimeManipulatorConfig
  ) {}

  async synchronize() {
    this.device.addRealtimeListener((_peer, message) => {
      console.log(message);
    });
    this.device.startListeningToRealtimeDataStream(
      this.config.currentJointStateStream
    );
  }
}
