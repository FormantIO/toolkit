import { afterEach, beforeEach, describe, it, vi, expect } from "vitest";

import { AuthenticationStore } from "./AuthenticationStore";

function fakeJwt(payload: Record<string, any>) {
  const header = { typ: "JWT", alg: "none" };
  const encoded = (data: Record<string, any>) =>
    Buffer.from(JSON.stringify(data)).toString("base64url");
  return `${encoded(header)}.${encoded(payload)}.`;
}

describe("AuthenticationStore", () => {
  it("should login with a token", async () => {
    const authStore = new AuthenticationStore({
      apiUrl: "https://vitest.invalid",
      addAccessTokenRefreshListener: vi.fn(),
      refreshAuthToken: vi.fn(),
    });

    expect(authStore.isAuthenticated()).toBe(false);
    expect(authStore.token).toBeUndefined();
    expect(authStore.currentUser).toBeUndefined();
    expect(authStore.currentOrganization).toBeUndefined();

    await authStore.loginWithToken(fakeJwt({}));

    expect(authStore.isAuthenticated()).toBe(true);
    expect(authStore.token).toBeDefined();
  });

  it("should extract the organization from the token", async () => {
    const authStore = new AuthenticationStore({
      apiUrl: "https://vitest.invalid",
      addAccessTokenRefreshListener: vi.fn(),
      refreshAuthToken: vi.fn(),
    });

    await authStore.loginWithToken(
      fakeJwt({
        "custom:organization_id": "00000000-0000-0000-0000-000000000000",
      })
    );

    expect(authStore.currentOrganization).toBe(
      "00000000-0000-0000-0000-000000000000"
    );
  });

  describe("share tokens", () => {
    it("should extract the organization from the token", async () => {
      const authStore = new AuthenticationStore({
        apiUrl: "https://vitest.invalid",
        addAccessTokenRefreshListener: vi.fn(),
        refreshAuthToken: vi.fn(),
      });

      await authStore.loginWithToken(
        fakeJwt({
          "formant:claims": {
            type: "share",
            organizationId: "00000000-0000-0000-0000-000000000000",
          },
        })
      );

      expect(authStore.currentOrganization).toBe(
        "00000000-0000-0000-0000-000000000000"
      );
    });
  });

  describe("waiting for auth", () => {
    it("should immediately resolve if already logged in", async () => {
      const authStore = new AuthenticationStore({
        apiUrl: "https://vitest.invalid",
        addAccessTokenRefreshListener: vi.fn(),
        refreshAuthToken: vi.fn(),
      });

      await authStore.loginWithToken(fakeJwt({}));
      await expect(authStore.waitTilAuthenticated()).resolves.ok;
    });

    it("should wait until the auth has been resolved", async () => {
      const authStore = new AuthenticationStore({
        apiUrl: "https://vitest.invalid",
        addAccessTokenRefreshListener: vi.fn(),
        refreshAuthToken: vi.fn(),
      });

      const waiting = authStore.waitTilAuthenticated();

      const timeout = (ms: number) =>
        new Promise((_, reject) => setTimeout(() => reject("pending"), ms));

      await expect(Promise.race([waiting, timeout(100)])).rejects.toThrow(
        "pending"
      );

      await authStore.loginWithToken(fakeJwt({}));

      await expect(waiting).resolves.ok;
    });
  });

  describe("refreshing tokens", () => {
    beforeEach(() => {
      vi.useFakeTimers();
    });

    afterEach(() => {
      vi.restoreAllMocks();
    });

    it("should schedule a refresh after one hour", async () => {
      const addAccessTokenRefreshListener = vi.fn();
      const refreshAuthToken = vi.fn();
      const authStore = new AuthenticationStore({
        apiUrl: "https://vitest.invalid",
        addAccessTokenRefreshListener,
        refreshAuthToken,
      });

      await authStore.loginWithToken(fakeJwt({}));

      expect(authStore.isAuthenticated()).toBe(true);
      await authStore.listenForRefresh();

      expect(refreshAuthToken).not.toHaveBeenCalled();
      // advance by an hour
      vi.advanceTimersByTime(60 * 60 * 1000);
      expect(refreshAuthToken).toHaveBeenCalled();
    });
  });
});
