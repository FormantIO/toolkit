import { describe, it, vi, expect } from "vitest";
import type { RtcClient, IRtcClientConfiguration } from "@formant/realtime-sdk";

import { RtcClientPool } from "./RtcClientPool";

describe("RtcClientPool", () => {
  it("should release when no more poxies are active", () => {
    const singleton = {
      shutdown: vi.fn(),
    } as unknown as RtcClient;

    const pool = new RtcClientPool({
      createClient() {
        return singleton;
      },
    });

    const client1 = pool.get(() => {});
    expect(singleton.shutdown).not.toBeCalled();
    const client2 = pool.get(() => {});
    expect(singleton.shutdown).not.toBeCalled();
    client1.release();
    expect(singleton.shutdown).not.toBeCalled();
    client2.release();
    expect(singleton.shutdown).toBeCalled();
  });

  it("should issue different proxies for each get()", () => {
    const singleton = {
      shutdown: vi.fn(),
    } as unknown as RtcClient;

    const pool = new RtcClientPool({
      createClient() {
        return singleton;
      },
    });

    const client1 = pool.get(() => {});
    const client2 = pool.get(() => {});

    expect(client1).not.toBe(client2);
  });

  it("should proxy calls to the singleton", () => {
    const singleton = {
      isReady: vi.fn(),
    } as unknown as RtcClient;

    const pool = new RtcClientPool({
      createClient() {
        return singleton;
      },
    });

    const client = pool.get(() => {});
    client.isReady();
    expect(singleton.isReady).toHaveBeenCalled();
  });

  it("should not be able to shutdown a proxy", () => {
    const singleton = {
      shutdown: vi.fn(),
    } as unknown as RtcClient;

    const pool = new RtcClientPool({
      createClient() {
        return singleton;
      },
    });

    const client = pool.get(() => {});
    expect(() => client.shutdown()).toThrowError(
      "shutdown not allowed for pooled client"
    );
    expect(singleton.shutdown).not.toBeCalled();
  });

  it("should call all the handlers that are listening", () => {
    let emit: IRtcClientConfiguration["receive"] = vi.fn();
    const pool = new RtcClientPool({
      createClient(receive) {
        emit = receive;
        return {} as any;
      },
    });

    const handle1 = vi.fn();
    pool.get(handle1);

    const handle2 = vi.fn();
    pool.get(handle2);

    const zeroUuid = "00000000-0000-0000-0000-000000000000";
    const fakeMessage = {} as unknown as any;

    emit(zeroUuid, fakeMessage);

    expect(handle1).toHaveBeenCalledWith(zeroUuid, fakeMessage);
    expect(handle2).toHaveBeenCalledWith(zeroUuid, fakeMessage);
  });

  it("should not call handle if been releases", () => {
    let emit: IRtcClientConfiguration["receive"] = vi.fn();
    const pool = new RtcClientPool({
      createClient(receive) {
        emit = receive;
        return {
          shutdown: vi.fn(),
        } as any;
      },
    });

    const handle1 = vi.fn();
    const client1 = pool.get(handle1);

    const handle2 = vi.fn();
    const client2 = pool.get(handle2);

    const zeroUuid = "00000000-0000-0000-0000-000000000000";
    const fakeMessage = {} as unknown as any;

    emit(zeroUuid, fakeMessage);

    expect(handle1).toHaveBeenLastCalledWith(zeroUuid, fakeMessage);
    expect(handle2).toHaveBeenLastCalledWith(zeroUuid, fakeMessage);

    handle1.mockReset();
    handle2.mockReset();

    client2.release();

    emit(zeroUuid, fakeMessage);

    expect(handle1).toHaveBeenLastCalledWith(zeroUuid, fakeMessage);
    expect(handle2).not.toHaveBeenCalled();

    handle1.mockReset();
    handle2.mockReset();

    client1.release();

    emit(zeroUuid, fakeMessage);

    expect(handle1).not.toHaveBeenCalled();
    expect(handle2).not.toHaveBeenCalled();
  });

  it("should warn about releasing multiple times", () => {
    const pool = new RtcClientPool({
      createClient() {
        return {
          shutdown: vi.fn(),
        } as any;
      },
    });

    const client = pool.get(vi.fn());
    expect(client.release()).resolves.toBe(true);
    expect(client.release()).resolves.toBe(false);
  });
});
