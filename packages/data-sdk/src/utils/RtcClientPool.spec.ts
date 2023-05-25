import { describe, it, vi, expect, beforeEach, afterEach } from "vitest";
import type { RtcClient, IRtcClientConfiguration } from "@formant/realtime-sdk";

import { RtcClientPool } from "./RtcClientPool";

describe("RtcClientPool", () => {
  it("should think it has a shutdown() method", () => {
    const pool = new RtcClientPool({
      createClient() {
        return {
          shutdown: vi.fn(),
        } as unknown as RtcClient;
      },
    });

    const client = pool.get(() => {});
    expect("shutdown" in client).toBe(true);
    expect(client.shutdown).toBeTypeOf("function");
  });

  it("should shutdown() when no more proxies are active", async () => {
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
    await client1.shutdown();
    expect(singleton.shutdown).not.toBeCalled();
    await client2.shutdown();
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

  it("should not call handle if been releases", async () => {
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

    await client2.shutdown();

    emit(zeroUuid, fakeMessage);

    expect(handle1).toHaveBeenLastCalledWith(zeroUuid, fakeMessage);
    expect(handle2).not.toHaveBeenCalled();

    handle1.mockReset();
    handle2.mockReset();

    await client1.shutdown();

    emit(zeroUuid, fakeMessage);

    expect(handle1).not.toHaveBeenCalled();
    expect(handle2).not.toHaveBeenCalled();
  });

  it("should warn about releasing multiple times", async () => {
    const pool = new RtcClientPool({
      createClient() {
        return {
          shutdown: vi.fn(),
        } as any;
      },
    });

    const client = pool.get(vi.fn());
    await expect(client.shutdown()).resolves.toBe(true);
    await expect(client.shutdown()).resolves.toBe(false);
  });

  describe("with ttl", () => {
    beforeEach(() => {
      vi.useFakeTimers();
    });

    afterEach(() => {
      vi.runOnlyPendingTimers();
      vi.useRealTimers();
    });

    it("should wait the duration before tearing down the singleton", async () => {
      const shutdown = vi.fn();
      const pool = new RtcClientPool({
        ttlMs: 1_000,
        createClient: () => ({ shutdown } as any),
      });

      await expect(pool.get().shutdown()).resolves.toBe(true);
      vi.advanceTimersByTime(100);
      expect(shutdown).not.toBeCalled();
      vi.advanceTimersByTime(875);
      expect(shutdown).not.toBeCalled();
      vi.advanceTimersByTime(50);
      expect(shutdown).toBeCalled();
    });

    it("should not release the singleton if another consumer allocates a reference", async () => {
      const shutdown = vi.fn();
      const pool = new RtcClientPool({
        ttlMs: 1_000,
        createClient: () => ({ shutdown } as any),
      });

      await pool.get().shutdown();
      vi.advanceTimersByTime(975);
      expect(shutdown).not.toBeCalled();

      const client = pool.get();
      vi.advanceTimersByTime(50);
      expect(shutdown).not.toBeCalled();
      vi.runAllTimers();
      expect(shutdown).not.toBeCalled();
      await expect(client.shutdown()).resolves.toBe(true);
      await vi.advanceTimersByTimeAsync(1000);
      expect(shutdown).toBeCalled();
    });
  });
});
