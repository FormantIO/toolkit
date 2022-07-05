import { Authentication, Fleet, KeyValue } from "@formant/data-sdk";
import "@formant/ui-sdk-joystick";
import "@formant/ui-sdk-realtime-player";
import { RealtimePlayer } from "@formant/ui-sdk-realtime-player";
import "./style.css";

const loadVideo = async (defaultCamera: number) => {
  (el("formant-realtime-player") as RealtimePlayer).drawer.start();
  let indicator = document.getElementById("loading-indicator");
  try {
    if (await Authentication.waitTilAuthenticated()) {
      const device = await Fleet.getCurrentDevice();

      await device.startRealtimeConnection();
      device.addRealtimeListener((_peerId: any, message: any) => {
        (el("formant-realtime-player") as RealtimePlayer).drawVideoFrame(
          message.payload.h264VideoFrame
        );
      });
      let videoStreams = await device.getRealtimeVideoStreams();
      let settingsIcon = document.getElementById("settings");
      let settingsBlock = document.getElementById("video-settings");

      settingsIcon!.style.display = "flex";
      settingsIcon?.addEventListener("click", () => {
        settingsBlock!.style.display = "flex";
      });
      renderCameraOptions(videoStreams);

      device.startListeningToRealtimeVideo(videoStreams[defaultCamera]);
      setTimeout(() => {
        indicator!.style.display = "none";
      }, 3000);
      el("formant-realtime-player").style.display = "block";
    }
  } catch (e) {
    log((e as Error).message);
  }
};

function saveDefaultCamera() {
  const videoSource = document.querySelectorAll("p.camera-option");
  videoSource.forEach((video) => {
    video.addEventListener("click", (event) => {
      const camera = (event.target as HTMLParagraphElement).id;
      KeyValue.set("defaultVideoStream", camera.at(-1)!);
      let settingsBlock = document.getElementById("video-settings");
      settingsBlock!.style.display = "none";
      location.reload();
    });
  });
}

function renderCameraOptions(videoStreams: any[]) {
  videoStreams.forEach((_, idx) => {
    $(".camera-options").append(
      `<p id=${`video-${idx}`} class="camera-option">${_.name}</p>`
    );
  });
}

function log(msg: string) {
  el("#log").innerHTML = msg + "<br>" + el("#log").innerHTML;
}

function el(selector: string) {
  return document.querySelector(selector) as HTMLElement;
}

el("formant-realtime-player").addEventListener("click", () => {
  let settingsBlock = document.getElementById("video-settings");
  settingsBlock!.style.display = "none";
});

document.body.style.display = "block";

const getDefaultCamera = async () => {
  let defaultCamera = 0;
  if (await Authentication.waitTilAuthenticated()) {
    try {
      defaultCamera = parseInt(await KeyValue.get("defaultVideoStream"));
    } catch (error) {
      defaultCamera = 0;
    }
  }
  return defaultCamera;
};

const main = async () => {
  const defaultCamera = await getDefaultCamera();
  await loadVideo(defaultCamera);
  saveDefaultCamera();
};
main();
