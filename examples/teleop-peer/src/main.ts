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

    // connect directly to a device (agent) via its IP.
    // NOTE: you must set the FORMANT_AGENT_IP ENV on the agent to an IP reachable by this UI.
    // you can use "0.0.0.0" if you would like to listen on all interfaces
    const device = await Fleet.getPeerDevice("http://10.4.4.182:5502");

    // start connecting to realtime and get videos and start one
    log("Currently looking at <b>" + device.id + "</b>");
    log("Getting a realtime connection ... ");
    await device.startRealtimeConnection();
    device.addRealtimeListener((_peerId: any, message: any) => {
      (el("formant-realtime-player") as RealtimePlayer).drawVideoFrame(
        message.payload.h264VideoFrame
      );
    });
    let videoStreams = await device.getRealtimeVideoStreams();
    log("Video streams: " + JSON.stringify(videoStreams));
    log("Starting to listen to video stream " + videoStreams[0].name + " ... ");
    device.startListeningToRealtimeVideo(videoStreams[0]);

    // show the player
    el("formant-realtime-player").style.display = "block";

    // show joysticks and connect them up
    let j = await device.createCustomDataChannel("joystick");
    el("formant-joystick").style.display = "block";
    el("formant-joystick").addEventListener("joystick", (e) => {
      const ce = e as CustomEvent;
      j.send(JSON.stringify(ce.detail));
    });
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
