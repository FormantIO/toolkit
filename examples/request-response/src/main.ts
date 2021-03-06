import { Authentication, Fleet } from "@formant/data-sdk";
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
                    const data = {
                        message: "Hello, world.",
                    };
                    log(`Sending request: ${JSON.stringify(data)}`);
                    const response = await requestChannel.request(
                        JSON.stringify(data)
                    );
                    log(`Response received: ${response}`); // Successfully received response
                } catch (e: any) {
                    if (e.name === "TimeoutError") {
                        log("Handling timeout..."); // Timeout
                    } else if (e.name === "AdapterError") {
                        // An error occurred in the adapter, e.g. a Python exception
                        log(
                            `Error in the adapter request handler: ${e.message}`
                        );
                        throw e;
                    } else {
                        throw e;
                    }
                }
            }
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
