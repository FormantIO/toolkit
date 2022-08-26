import {
  DataChannel,
  DataChannelErrorListener,
  DataChannelListener,
} from "./main";
import { delay } from "../../common/delay";
import { IRealtimeDevice } from "./Device";
import { defined } from "../../common/defined";

// AdapterError -> An error occurred when handling the request on the adapter.
// TimeoutError -> The request did not receive a response within the timeout period.

abstract class RequestDataChannel {
  protected channel: undefined | DataChannel;
  protected requestIdToResponseMap = new Map<string, any>();
  constructor(
    protected device: IRealtimeDevice,
    protected channel_name: string,
    protected timeout: number
  ) {}

  addOpenListener(listener: DataChannelListener) {
    defined(this.channel, "channel not initalized").addOpenListener(listener);
  }

  removeOpenListener(listener: DataChannelListener) {
    defined(this.channel, "channel not initalized").removeOpenListener(
      listener
    );
  }

  addCloseListener(listener: DataChannelListener) {
    defined(this.channel, "channel not initalized").addCloseListener(listener);
  }

  removeCloseListener(listener: DataChannelListener) {
    defined(this.channel, "channel not initalized").removeCloseListener(
      listener
    );
  }

  addErrorListener(listener: DataChannelErrorListener) {
    defined(this.channel, "channel not initalized").addErrorListener(listener);
  }

  removeErrorListener(listener: DataChannelErrorListener) {
    defined(this.channel, "channel not initalized").removeErrorListener(
      listener
    );
  }
}

export class BinaryRequestDataChannel extends RequestDataChannel {
  private RESPONSE_SUCCESS_BYTE = 0;
  private decoder = new TextDecoder();

  /*
    Request binary payload layout:
        16-bytes         arbitrary-length
    [     ID       ] [        PAYLOAD         ]

    Response binary payload layout:
            1-byte               16-bytes         arbitrary-length
    [ SUCCESS OR ERROR BYTE ] [     ID       ] [        PAYLOAD         ]
    */

  generateBinaryId() {
    // attach binary ID as 16-byte header in binary messages
    const id = new Uint8Array(16);
    for (let i = 0; i < id.length; i++) {
      id[i] = Math.floor(Math.random() * 256);
    }
    return id;
  }

  async initialize() {
    this.channel = await this.device.createCustomDataChannel(this.channel_name);

    this.channel.addBinaryListener((message) => {
      const binaryId = message.slice(0, 16);

      const id = binaryId.toString();
      if (id.length === 0) {
        throw new Error("Invalid response");
      }

      const response = message.slice(16);
      if (response.length === 0) {
        throw new Error("Invalid response");
      }

      // only add to the map if there is an active request
      if (this.requestIdToResponseMap.has(id)) {
        this.requestIdToResponseMap.set(id, response);
      }
    });
  }

  async request(data: Uint8Array) {
    if (!this.channel) {
      await this.initialize();
    }
    if (!this.channel) {
      throw new Error("Failed to create channel");
    }
    const { channel, requestIdToResponseMap, timeout } = this;
    await channel.waitTilReady();

    const binaryId = this.generateBinaryId();
    const id = binaryId.toString();
    requestIdToResponseMap.set(id, true); // true signifies an active request
    channel.sendBinary(new Uint8Array([...binaryId, ...data]));

    // Wait for the response to come back.
    const start = new Date().getTime();
    while (new Date().getTime() < start + timeout) {
      await delay(50);
      if (requestIdToResponseMap.has(id)) {
        const response = requestIdToResponseMap.get(id);
        if (response !== true) {
          requestIdToResponseMap.delete(id);
          const success = response[0] === this.RESPONSE_SUCCESS_BYTE;
          const payload = response.slice(1);
          if (success) {
            return payload;
          } else {
            console.error({
              name: "AdapterError",
              message: this.decoder.decode(payload),
            });
            throw new Error("Binary request datachannel adapter error");
          }
        }
      }
    }

    requestIdToResponseMap.delete(id);
    console.error({
      name: "TimeoutError",
      message: `Request timed out after ${timeout / 1000.0} seconds`,
    });
    throw new Error("Binary request data channel request timed out");
  }
}

export class TextRequestDataChannel extends RequestDataChannel {
  generateTextId() {
    return (
      Math.random().toString(36).substring(2) +
      "-" +
      Math.random().toString(36).substring(2)
    );
  }

  async initialize() {
    this.channel = await this.device.createCustomDataChannel(this.channel_name);

    this.channel.addListener((message) => {
      const response = JSON.parse(message);
      const { id, data, error } = response;
      if (!id) {
        throw new Error("Invalid response");
      }
      if (!data && !error) {
        throw new Error("Invalid response");
      }
      // only add to the map if there is an active request
      if (this.requestIdToResponseMap.has(id)) {
        this.requestIdToResponseMap.set(id, response);
      }
    });
  }

  async request(data: string) {
    if (!this.channel) {
      await this.initialize();
    }
    if (!this.channel) {
      throw new Error("Failed to create channel");
    }
    const { channel, requestIdToResponseMap, timeout } = this;
    await channel.waitTilReady();

    const id = this.generateTextId();
    requestIdToResponseMap.set(id, true); // true signifies an active request
    channel.send(
      JSON.stringify({
        id,
        data,
      })
    );

    // Wait for the response to come back.
    const start = new Date().getTime();
    while (new Date().getTime() < start + timeout) {
      await delay(50);
      if (requestIdToResponseMap.has(id)) {
        const response = requestIdToResponseMap.get(id);
        if (response !== true) {
          requestIdToResponseMap.delete(id);
          const { data, error } = response;
          if (data) {
            return data;
          }
          if (error) {
            console.error({
              name: "AdapterError",
              message: error,
            });
            throw new Error("Text request datachannel adapter error");
          }
        }
      }
    }

    requestIdToResponseMap.delete(id);
    console.error({
      name: "TimeoutError",
      message: `Request timed out after ${timeout / 1000.0} seconds`,
    });
    throw new Error("Text request datachannel request timed out");
  }
}
