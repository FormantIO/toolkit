import { Authentication, Fleet, KeyValue } from "@formant/data-sdk";
import "@formant/ui-sdk-joystick";
import "@formant/ui-sdk-realtime-player";
import { RealtimePlayer } from "@formant/ui-sdk-realtime-player";
import "./style.css";

const loadVideo = async (defaultCamera: number | string) => {
  const player = el("formant-realtime-player") as RealtimePlayer;
  player.drawer.start();
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
      const videoStreams = await device.getRealtimeVideoStreams();
      const settingsIcon = document.getElementById("settings")!;
      const settingsBlock = document.getElementById("video-settings")!;

      settingsIcon.style.display = "flex";
      settingsIcon.addEventListener("click", () => {
        settingsBlock.style.display = "flex";
      });
      renderCameraOptions(videoStreams);
      const foundCamera = (_: { name: string }) => _.name === defaultCamera;

      device.startListeningToRealtimeVideo(
        videoStreams[
          videoStreams.findIndex(foundCamera) > -1
            ? videoStreams.findIndex(foundCamera)
            : 0
        ]
      );
      setTimeout(() => {
        indicator!.style.display = "none";
      }, 3000);
      player.style.display = "block";
    }
  } catch (e) {
    log((e as Error).message);
  }
};

function saveDefaultCamera() {
  const videoSource = document.querySelectorAll("p.camera-option");
  videoSource.forEach((video) => {
    video.addEventListener("click", (event) => {
      let camera: string | string[] = (event.target as HTMLParagraphElement).id;
      camera = camera.split("/");
      camera = camera.join(" ");
      KeyValue.set("defaultVideoStream", camera);
      let settingsBlock = document.getElementById("video-settings");
      settingsBlock!.style.display = "none";
      location.reload();
    });
  });
}

function renderCameraOptions(videoStreams: any[]) {
  videoStreams.forEach((_) => {
    let _id: string | string[] = _.name.trim().split(/\s+/);
    _id = (_id as string[]).join("/");
    $(".camera-options").append(
      `<p id=${`${_id}`} class="camera-option">${_.name}</p>`
    );
  });
}

const log = (msg: string) =>
  el("#log").prepend(msg, document.createElement("br"));

const el = (selector: string): HTMLElement => {
  const match = document.querySelector(selector);
  if (!(match instanceof HTMLElement)) {
    throw new Error(`No element found: "${selector}"`);
  }
  return match;
};

el("formant-realtime-player").addEventListener("click", () => {
  let settingsBlock = document.getElementById("video-settings");
  settingsBlock!.style.display = "none";
});

document.body.style.display = "block";

const getDefaultCamera = async () => {
  let defaultCamera: number | string | string[] = 0;
  if (await Authentication.waitTilAuthenticated()) {
    try {
      defaultCamera = await KeyValue.get("defaultVideoStream");
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
