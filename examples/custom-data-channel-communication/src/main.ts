import { Authentication, Fleet } from "@formant/data-sdk";
import "@formant/ui-sdk-joystick";
import "@formant/ui-sdk-realtime-player";
import "./style.css";

// When the user clicks "connect"
el("button").addEventListener("click", async () => {
    try {
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

        // connect to realtime
        log("Currently looking at <b>" + device.name + "</b>");
        log("Getting a realtime connection ... ");
        await device.startRealtimeConnection();

        // Create the custom data channel
        const channel = await device.createCustomDataChannel("example_channel");

        // Send data from the custom web application to the robot every 200ms
        setInterval(() => {
            channel.send(
                JSON.stringify({
                    value: `JSON message from the web application sent at ${Date.now()}`,
                })
            );
        }, 200);

        // Listen to data from the robot and log it to the screen
        channel.addListener((message) => {
            log(`Received JSON message from the robot: ${message}`);
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
