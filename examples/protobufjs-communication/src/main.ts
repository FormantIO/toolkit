import { Authentication, Fleet } from "@formant/data-sdk";
import "@formant/ui-sdk-joystick";
import "@formant/ui-sdk-realtime-player";
import "./style.css";
import * as protobuf from "protobufjs";

// When the user clicks "connect"
el("button").addEventListener("click", async () => {
    try {
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

        // Send binary data from the custom web application to the robot every 200ms
        setInterval(() => {
            // 4. Send the Uint8Array to python
            channel.sendBinary(buffer);
        }, 200);

        // Add a binary listener
        channel.addBinaryListener((message) => {
            // 5. Decode Uint8Arrays received from python
            const decodedPerson = Person.decode(message);
            console.log("received back", decodedPerson);
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
