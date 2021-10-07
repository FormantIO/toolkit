import {
  Authentication,
  Fleet,
} from "https://cdn.jsdelivr.net/npm/@formant/data-sdk/dist/data-sdk.es.js";

await Authentication.login("sam@robot.io", "abc123");

const allDevices = await Fleet.getDevices();

console.log(allDevices);
