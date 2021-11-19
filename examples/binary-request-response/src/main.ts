import { Authentication, Fleet } from "@formant/data-sdk";
import "./style.css";
import * as protobuf from "protobufjs";

// When the user clicks "connect"
el("button").addEventListener("click", async () => {
    try {
        // Hide intro
        el("section").style.display = "none";
        el("#log").style.display = "block";

        // 1. Load proto types
        const protos = await protobuf.load("protos/example.proto");
        const Person = protos.lookupType("example.Person");

        // 2. Create a protobufjs object
        const person = Person.create({
            id: 1234,
            name: "Ali",
        });

        // 3. Encode the protobufjs object into a Uint8Array
        const buffer = Person.encode(person).finish();

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

        // Create binary request channel for request : response framework
        const requestChannel = device.createCustomBinaryRequestDataChannel(
            "request_response_channel"
        );

        // Send a request when spacebar is pressed
        // Receive a response or timeout error
        document.addEventListener("keydown", async (e) => {
            if (e.code === "Space") {
                try {
                    // Send and receive Uint8Array data
                    log("Sending protobuf buffer to Python");
                    console.log("Sending:", Person.decode(buffer));
                    const response = await requestChannel.request(buffer);
                    log(
                        "Console logging response protobuf object back from Python"
                    );
                    console.log("Received:", Person.decode(response));
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
