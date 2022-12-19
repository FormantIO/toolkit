import { Authentication, Fleet } from "@formant/data-sdk";
import "@formant/ui-sdk-joystick";
import "@formant/ui-sdk-realtime-player";
import { RealtimePlayer } from "@formant/ui-sdk-realtime-player";
import "./style.css";

(el("formant-realtime-player") as RealtimePlayer).drawer.start();

// When the user clicks "connect"
el("button").addEventListener("click", async () => {
  try {
    // hide intro
    el("section").style.display = "none";
    el("#log").style.display = "block";

    // start connecting
    log("Waiting for authentication ...");
    const username = (el("#username") as HTMLInputElement).value;
    const password = (el("#password") as HTMLInputElement).value;
    if (!(await Authentication.login(username, password))) {
      log("Not Authenticated");
      return;
    }

    // get current device from url context
    const device = await Fleet.getCurrentDevice();

    // start connecting to realtime and get videos and start one
    log("Currently looking at <b>" + device.name + "</b>");
    log("Getting a realtime connection ... ");
    await device.startRealtimeConnection();
    device.addRealtimeListener((_peerId: any, message: any) => {
      console.log(message);
      (el("formant-realtime-player") as RealtimePlayer).drawVideoFrame(
        message.payload.h264VideoFrame
      );
    });
    let videoStreams = await device.getRealtimeVideoStreams();
    log("Video streams: " + JSON.stringify(videoStreams));
    log("Starting to listen to video stream " + videoStreams[0].name + " ... ");
    debugger;
    device.startListeningToRealtimeVideo(videoStreams[0]);

    // show the player
    el("formant-realtime-player").style.display = "block";
  } catch (e) {
    log((e as Error).message);
  }
});

function log(msg: string) {
  el("#log").innerHTML = msg + "<br>" + el("#log").innerHTML;
}

function el(selector: string) {
  return document.querySelector(selector) as HTMLElement;
}

document.body.style.display = "block";
