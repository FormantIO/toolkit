import { Authentication, Fleet } from "@formant/data-sdk";
import * as UiCore from "@formant/ui-sdk-realtime-player-core";

import React, { Component } from "react";

export class RealtimePlayer extends Component {
  private h264BytestreamCanvasDrawer: UiCore.H264BytestreamCanvasDrawer;

  public constructor(props: any) {
    super(props);
    this.h264BytestreamCanvasDrawer = new UiCore.H264BytestreamCanvasDrawer(
      this.setWebglYUVSupported,
      this.setWarningText,
      this.handleCanvasDrawerWarning
    );
  }

  public async componentDidMount() {
    await this.start();
  }

  public async componentWillUnmount() {
    await this.stop();
  }

  public render() {
    return (
      <canvas
        ref={(_) => this.h264BytestreamCanvasDrawer.setCanvas(_ || undefined)}
      />
    );
  }

  private start = async () => {
    await Authentication.waitTilAuthenticated();

    const device = await Fleet.getCurrentDevice();
    await device.startRealtimeConnection();

    const videoStreams = await device.getRealtimeVideoStreams();
    const videoStream = videoStreams[0];

    device.addRealtimeListener((_, message) => {
      if (message.header.stream.streamName === videoStream.name) {
        this.h264BytestreamCanvasDrawer.receiveEncodedFrame(
          message.payload.h264VideoFrame
        );
      }
    });
    await device.startListeningToRealtimeVideo(videoStream);
    this.h264BytestreamCanvasDrawer.start();
  };
  private stop = async () => {
    const device = await Fleet.getCurrentDevice();

    const videoStreams = await device.getRealtimeVideoStreams();
    const videoStream = videoStreams[0];

    await device.stopListeningToRealtimeVideo(videoStream);

    this.h264BytestreamCanvasDrawer.stop();
  };

  private setWebglYUVSupported = (value?: boolean) => {
    // Do something with the error that webglyuv is not supported
  };
  private setWarningText = (value?: string) => {
    // Do something with the warning text of why video is not starting
  };
  private handleCanvasDrawerWarning = (value: any) => {
    // Do something with the why video isn't performing well
  };
}
