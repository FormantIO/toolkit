import { Authentication, Fleet } from "@formant/data-sdk";
import "@formant/ui-sdk-joystick";
import "@formant/ui-sdk-realtime-player";
import "./style.css";

// When the user clicks "connect"
el("button").addEventListener("click", async () => {
    try {
        // Hide intro
        el("section").style.display = "none";
        el("#log").style.display = "block";

        // Start connecting
        log("Waiting for authentication ...");
        if (!(await Authentication.waitTilAuthenticated())) {
            log("Not Authenticated");
            return;
        }

        // Get current device from url context
        const device = await Fleet.getCurrentDevice();

        // Connect to realtime
        log("Currently looking at <b>" + device.name + "</b>");
        log("Getting a realtime connection ... ");
        await device.startRealtimeConnection();

        // Create RequestChannel for request : response framework
        const requestChannel = device.createCustomRequestDataChannel(
            "request_response_channel"
        );

        // Send a request when spacebar is pressed
        // Receive a response or timeout error
        document.addEventListener("keydown", async (e) => {
            if (e.code === "Space") {
                try {
                    const response = await requestChannel.request(
                        JSON.stringify({
                            message: "Hello World",
                        })
                    );
                    console.log("Response received:", response); // Successfully received response
                } catch (e: any) {
                    console.log("Error:", e);
                    if (e.name === "TimeoutError") {
                        console.log("Handling timeout..."); // Timeout
                    } else {
                        throw e; // Non-timeout error; throw again
                    }
                }
            }
        });
    } catch (e) {
        console.log("Error happened!", e);
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
