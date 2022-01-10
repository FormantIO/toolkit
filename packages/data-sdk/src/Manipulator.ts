import {
  Device,
  RealtimeDataStream,
  IJointState,
  RealtimeMessage,
} from "./Device";

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
  currentListeners: ((js: IJointState) => void)[] = [];
  constructor(
    private device: Device,
    private config: RealtimeManipulatorConfig
  ) {}

  async synchronize() {
    this.device.addRealtimeListener(this.onRealtimeMessage);
    this.device.startListeningToRealtimeDataStream(
      this.config.currentJointStateStream
    );
  }

  async desynchronize() {
    this.device.removeRealtimeListener(this.onRealtimeMessage);
    this.device.stopListeningToRealtimeDataStream(
      this.config.currentJointStateStream
    );
  }

  onRealtimeMessage = (_peerId: string, message: RealtimeMessage) => {
    if (message.payload.jointState) {
      this.currentListeners.forEach((listener) =>
        listener(message.payload.jointState)
      );
    }
  };

  async addCurrentJointStateListener(listener: (js: IJointState) => void) {
    this.currentListeners.push(listener);
  }
}
