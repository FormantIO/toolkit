import {
  RealtimeDataStream,
  IJointState,
  RealtimeMessage,
} from "./devices/device.types";
import { IRealtimeSubscriber } from "./devices/IRealtimeSubscriber";

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
    private device: IRealtimeSubscriber,
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
      this.currentListeners.forEach((listener) => {
        if (message.payload.jointState) listener(message.payload.jointState);
      });
    }
  };

  async addCurrentJointStateListener(listener: (js: IJointState) => void) {
    this.currentListeners.push(listener);
  }
}
