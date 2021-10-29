import { Device, DataChannel } from "./main";
import { delay } from "../../common/delay";

function generateId() {
    return (
        Math.random().toString(36).substring(2) +
        "-" +
        Math.random().toString(36).substring(2)
    );
}

export class RequestDataChannel {
    private channel: undefined | DataChannel;
    private requestIdToResponseMap = new Map<string, any>();
    constructor(
        private device: Device,
        private channel_name: string,
        private timeout: number
    ) {}

    async initialize() {
        this.channel = await this.device.createCustomDataChannel(
            this.channel_name
        );
        this.channel.addListener((message) => {
            const { id, data } = JSON.parse(message);
            if (!id || !data) {
                throw new Error("Invalid response");
            }
            // only add to the map if there is an active request
            if (this.requestIdToResponseMap.has(id)) {
                this.requestIdToResponseMap.set(id, data);
            }
        });
    }

    async request(data: any) {
        if (!this.channel) {
            await this.initialize();
        }
        if (!this.channel) {
            throw new Error("Failed to create channel");
        }
        const { channel, requestIdToResponseMap, timeout } = this;

        await channel.waitTilReady();

        const id = generateId();

        // true signifies an active request
        requestIdToResponseMap.set(id, true);

        channel.send(
            JSON.stringify({
                id,
                data,
            })
        );

        const start = new Date().getTime();
        while (new Date().getTime() < start + timeout) {
            await delay(50);
            if (requestIdToResponseMap.has(id)) {
                const response = requestIdToResponseMap.get(id);
                if (response !== true) {
                    requestIdToResponseMap.delete(id);
                    return response;
                }
            }
        }

        requestIdToResponseMap.delete(id);
        throw {
            name: "TimeoutError",
            message: `Request timed out after ${timeout / 1000.0} seconds`,
        };
    }
}
