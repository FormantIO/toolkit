import { Authentication, Fleet } from "@formant/data-sdk";
import { PeerDevice } from "./PeerDevice";
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

    // get current device from url context
    const peer = new PeerDevice("http://127.0.0.1:5502");
    peer.id = await peer.getDeviceId();
    console.log(peer.getConfiguration())
    const device = peer
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

    const twistStreams = await device.getRealtimeTwistStreams()
    const twistStream = twistStreams[0]
    console.log(twistStream)
    log("Video streams: " + JSON.stringify(videoStreams));
    log("Starting to listen to video stream " + videoStreams[0].name + " ... ");
    device.startListeningToRealtimeVideo(videoStreams[0]);

    // show the player
    el("formant-realtime-player").style.display = "block";

    // show joysticks and connect them up
    el("formant-joystick").style.display = "block";
    el("formant-joystick").addEventListener("joystick", (e) => {
      const ce = e as CustomEvent;
      console.log(ce.detail)
      device.sendRealtimeMessage({
        header: {
          stream: {
            entityId: device.id,
            streamName: twistStream.name,
            streamType: "twist",
          },
          created: 0,
        },
        payload: {
          twist: {
            linear: {
              x: ce.detail.y,
              y: 0,
              z: 0,
            },
            angular: {
              x: 0,
              y: 0,
              z: ce.detail.x,
            },
          },
        },
      });   
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
