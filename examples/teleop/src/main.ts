import { Authentication, Fleet } from "@formant/data-sdk";
import "@formant/ui-sdk-joystick";
import "@formant/ui-sdk-realtime-player";
import { RealtimePlayer } from "@formant/ui-sdk-realtime-player";
import "./style.css";
import { delay } from "../../../packages/common/delay";

(el("formant-realtime-player") as RealtimePlayer).drawer.start();

// When the user clicks "connect"
el("button").addEventListener("click", async () => {
  // hide intro
  el("section").style.display = "none";
  el("#log").style.display = "block";

  // start connecting
  log("Waiting for authentication ...");
  if (!(await Authentication.waitTilAuthenticated())) {
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
    (el("formant-realtime-player") as RealtimePlayer).drawVideoFrame(
      message.payload.h264VideoFrame
    );
  });
  let videoStreams = await device.getRealtimeVideoStreams();
  log("Video streams: " + JSON.stringify(videoStreams));
  log("waiting...");
  await delay(1000);
  log("Starting to listen to video stream " + videoStreams[0].name + " ... ");
  device.startListeningToRealtimeVideo(videoStreams[0]);

  // show the player
  el("formant-realtime-player").style.display = "block";

  // show joysticks and connec them up
  el("formant-joystick").style.display = "block";
  el("formant-joystick").addEventListener("joystick", (e) => {
    const ce = e as CustomEvent;
    log(JSON.stringify(ce.detail));
  });
});

function log(msg: string) {
  el("#log").innerHTML = msg + "<br>" + el("#log").innerHTML;
}

function el(selector: string) {
  return document.querySelector(selector) as HTMLElement;
}

document.body.style.display = "block";
