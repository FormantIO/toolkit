import { afterEach, beforeEach, describe, it, vi, expect } from "vitest";

import { AuthenticationStore } from "./AuthenticationStore";

function fakeJwt(payload: Record<string, any>) {
  return [
    Buffer.from(JSON.stringify({ typ: "JWT", alg: "none" })).toString(
      "base64url"
    ),
    Buffer.from(JSON.stringify(payload)).toString("base64url"),
    "",
  ].join(".");
}

describe("AuthenticationStore", () => {
  it("should login with a token", () => {
    const authStore = new AuthenticationStore({
      apiUrl: "https://vitest.invalid",
      addAccessTokenRefreshListener: vi.fn(),
      refreshAuthToken: vi.fn(),
    });
    expect(authStore.isAuthenticated()).toBe(false);
    expect(authStore.token).toBeUndefined();
    expect(authStore.currentUser).toBeUndefined();
    expect(authStore.currentOrganization).toBeUndefined();

    authStore.loginWithToken(fakeJwt({}));

    expect(authStore.isAuthenticated()).toBe(true);
    expect(authStore.token).toBeDefined();
    console.log(authStore.token);
  });

  it("should extract the organization from the token", () => {
    const authStore = new AuthenticationStore({
      apiUrl: "https://vitest.invalid",
      addAccessTokenRefreshListener: vi.fn(),
      refreshAuthToken: vi.fn(),
    });
    authStore.loginWithToken(
      fakeJwt({
        "custom:organization_id": "00000000-0000-0000-0000-000000000000",
      })
    );

    expect(authStore.currentOrganization).toBe(
      "00000000-0000-0000-0000-000000000000"
    );
  });

  describe("refreshing tokens", () => {
    beforeEach(() => {
      vi.useFakeTimers();
    });

    afterEach(() => {
      vi.restoreAllMocks();
    });

    it("should schedule a refresh after one hour", () => {
      const addAccessTokenRefreshListener = vi.fn();
      const refreshAuthToken = vi.fn();
      const authStore = new AuthenticationStore({
        apiUrl: "https://vitest.invalid",
        addAccessTokenRefreshListener,
        refreshAuthToken,
      });

      authStore.loginWithToken(fakeJwt({}));

      expect(authStore.isAuthenticated()).toBe(true);
      authStore.listenForRefresh();

      expect(refreshAuthToken).not.toHaveBeenCalled();
      // advance by an hour
      vi.advanceTimersByTime(60 * 60 * 1000);
      expect(refreshAuthToken).toHaveBeenCalled();
    });
  });
});
