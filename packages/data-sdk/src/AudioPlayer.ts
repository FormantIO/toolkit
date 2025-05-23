import { IRtcStreamMessage } from "@formant/realtime-sdk";
import { Device } from "./devices/Device";
import { stringToArrayBuffer } from "./utils/stringToArrayBuffer";
import { fork } from "../../common/fork";
import { browser } from "../../common/browser";
import { RealtimeDataStream, RealtimeMessage } from "./devices/device.types";

const rtcAudioChunkStreamType = "audio-chunk";

export class AudioPlayer {
  private muted: boolean = false;

  public async play() {
    if (this.audioContext?.state === "suspended") {
      await this.audioContext?.resume();
    }
    this.muted = false;
  }

  public async pause() {
    await this.audioContext.suspend();
    this.muted = true;
  }

  private hasReceivedData: boolean = false;
  private audioContext: AudioContext;
  private chunks: Array<AudioBufferSourceNode> = [];
  private isPlaying: boolean = false;
  private startTime: number = 0;
  private lastChunkOffset: number = 0;
  private bufferSize: number = 3;

  public constructor(
    private device: Device,
    private stream: RealtimeDataStream
  ) {
    this.device.startListeningToRealtimeDataStream(stream);
    this.device.addRealtimeListener((_peerId: string, msg: RealtimeMessage) => {
      this.receive(msg);
    });

    if (browser() === "Safari" || browser() === "IE") {
      this.changeAudioWireFormat("wav");
    } else {
      this.changeAudioWireFormat("opus");
    }

    const AudioContext =
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      window.AudioContext || (window as any).webkitAudioContext;

    this.audioContext = new AudioContext();
  }

  public destroy() {
    this.device.stopListeningToRealtimeDataStream(this.stream);
  }

  public receive = async (msg: IRtcStreamMessage) => {
    const data = msg.payload.audioChunk?.chunk_data;
    if (!data) {
      return;
    }

    if (!this.hasReceivedData) {
      this.hasReceivedData = true;
    }

    const { audioContext, muted } = this;
    if (
      !audioContext ||
      msg.header.stream.streamType !== rtcAudioChunkStreamType ||
      muted !== false
    ) {
      return;
    }
    const arrayBuffer = stringToArrayBuffer(data);
    try {
      await audioContext.decodeAudioData(
        toArrayBuffer(arrayBuffer.buffer),
        this.scheduleChunk
      );
    } catch (error) {
      console.warn(
        "Error decoding audio buffer, changing audioWireFormat on agent",
        { error }
      );
      this.changeAudioWireFormat("wav");
    }
  };

  private scheduleChunk = (buffer: AudioBuffer) => {
    const { audioContext } = this;

    if (!audioContext) {
      return;
    }
    if (this.chunks.length > this.bufferSize || this.isPlaying === false) {
      this.chunks.forEach((c) => {
        c.stop();
      });
      this.isPlaying = false;
      this.chunks = [];
    }
    const chunk = this.createChunk(buffer);
    if (!chunk) {
      return;
    }
    if (!chunk.buffer) {
      return;
    }
    if (this.isPlaying === false) {
      this.startTime = audioContext.currentTime;
      this.lastChunkOffset = 0;
      this.isPlaying = true;
    }
    chunk.start(this.startTime + this.lastChunkOffset, 0, buffer.duration);
    this.lastChunkOffset += chunk.buffer.duration;
    this.chunks.push(chunk);
  };

  private createChunk(buffer: AudioBuffer) {
    const { audioContext } = this;
    if (!audioContext) {
      return;
    }
    const source = audioContext.createBufferSource();
    source.buffer = buffer;
    source.connect(audioContext.destination);
    source.loop = false;
    // eslint-disable-next-line @typescript-eslint/no-unused-vars
    source.onended = (_: Event) => {
      this.chunks.splice(this.chunks.indexOf(source), 1);
      if (this.chunks.length === 0) {
        this.isPlaying = false;
      }
    };

    return source;
  }

  private changeAudioWireFormat(newFormat: "wav" | "opus") {
    const { stream } = this;
    fork(
      (async () => {
        await this.device.changeStreamAudioType(stream.name, newFormat);
      })()
    );
  }
}

function toArrayBuffer(data: ArrayBufferLike): ArrayBuffer {
  // (1) If it’s already an ArrayBuffer, just return it.
  if (data instanceof ArrayBuffer) {
    return data;
  }

  // (2) If it’s a TypedArray or DataView, pull out the exact bytes.
  if (ArrayBuffer.isView(data)) {
    const view = data as ArrayBufferView;
    const buffer = view.buffer.slice(
      view.byteOffset,
      view.byteOffset + view.byteLength
    );
    if (buffer instanceof SharedArrayBuffer) {
      return new Uint8Array(data as SharedArrayBuffer).slice().buffer;
    }
    return buffer;
  }

  // (3) Otherwise it must be a SharedArrayBuffer — copy it into a new ArrayBuffer.
  //     We do this by wrapping in a Uint8Array and slicing.
  return new Uint8Array(data as SharedArrayBuffer).slice().buffer;
}
