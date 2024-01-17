import { Fleet } from "@formant/data-sdk";
import "@formant/ui-sdk-joystick";
import "@formant/ui-sdk-realtime-player";
import { RealtimePlayer } from "@formant/ui-sdk-realtime-player";
import "./style.css";


declare global {
  interface Window {
    setVideo: (num: number) => void;
  }
}

(el("formant-realtime-player") as RealtimePlayer).drawer.start();

// When the user clicks "connect"
el("button").addEventListener("click", async () => {
  try {
    // hide intro
    el("section").style.display = "none";
    el("#log").style.display = "block";
    // Just connect to whereever this is hosted on port 5502
    const host = window.location.origin.replace(/:[0-9]+$/, '');
    const device = await Fleet.getPeerDevice(`${host}:5502`);

    // start connecting to realtime and get videos and start one
    log("Currently looking at <b>" + device.id + "</b>");
    log("Getting a realtime connection ... ");
    const listener = (_peerId: any, message: any) => {
      (el("formant-realtime-player") as RealtimePlayer).drawVideoFrame(
        message.payload.h264VideoFrame
      );
    };
    await device.startRealtimeConnection();

    let videoStreams = await device.getRealtimeVideoStreams();
    let last: number;

    window.setVideo = async (num: number) => {
      if (last) {
        // await device.stopRealtimeConnection();
        await device.stopListeningToRealtimeVideo(videoStreams[last]);
        await device.removeRealtimeListener(listener);
      }
      log(`Starting to listen to video stream ${videoStreams[num].name}  ... `);
      await device.startListeningToRealtimeVideo(videoStreams[num]);
      await device.addRealtimeListener(listener);
      last = num;
    };

    log("Video streams: ");
    videoStreams.forEach((s, i) => {
      log(`<a onClick="setVideo(${i})" href="#">${i} ${s.name}</a>`);
    });

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
