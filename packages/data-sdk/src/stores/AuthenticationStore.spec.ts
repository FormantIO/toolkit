import { afterEach, beforeEach, describe, it, vi, expect } from "vitest";

import { AuthenticationStore } from "./AuthenticationStore";

function fakeJwt(payload: Record<string, any>) {
  const header = { typ: "JWT", alg: "none" };
  const encoded = (data: Record<string, any>) =>
    Buffer.from(JSON.stringify(data)).toString("base64url");
  return `${encoded(header)}.${encoded(payload)}.`;
}

const mockFetch = vi.fn();
global.fetch = mockFetch;

beforeEach(() => {
  mockFetch.mockClear();
});

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

  describe(".login()", () => {
    describe("basic login", () => {
      it("should handle successful login", async () => {
        const authStore = new AuthenticationStore({
          apiUrl: "https://vitest.invalid",
          addAccessTokenRefreshListener: vi.fn(),
          refreshAuthToken: vi.fn(),
        });

        mockFetch.mockImplementation((url) => {
          if (url.endsWith("/v1/admin/auth/login")) {
            return {
              status: 200,
              async json() {
                return {
                  authentication: {
                    accessToken: fakeJwt({
                      sub: "00000000-0000-0000-0000-000000000002",
                    }),
                    refreshToken: fakeJwt({}),
                    organizationId: "00000000-0000-0000-0000-000000000001",
                    userId: "00000000-0000-0000-0000-000000000002",
                  },
                };
              },
            };
          }

          if (
            url.endsWith("/v1/admin/users/00000000-0000-0000-0000-000000000002")
          ) {
            return {
              status: 200,
              async json() {
                return {};
              },
            };
          }

          return {
            status: 404,
          };
        });

        const result = await authStore.login("jim@starfleet.com", "jamestkirk");
        expect(result).toMatchObject({
          accessToken:
            "eyJ0eXAiOiJKV1QiLCJhbGciOiJub25lIn0.eyJzdWIiOiIwMDAwMDAwMC0wMDAwLTAwMDAtMDAwMC0wMDAwMDAwMDAwMDIifQ.",
          organizationId: "00000000-0000-0000-0000-000000000001",
          refreshToken: "eyJ0eXAiOiJKV1QiLCJhbGciOiJub25lIn0.e30.",
          userId: "00000000-0000-0000-0000-000000000002",
        });
        expect(authStore.currentUser).not.toBeUndefined();
      });
    });

    it("should handle a challege as a failure", async () => {
      const authStore = new AuthenticationStore({
        apiUrl: "https://vitest.invalid",
        addAccessTokenRefreshListener: vi.fn(),
        refreshAuthToken: vi.fn(),
      });

      mockFetch.mockImplementation((url) => {
        if (url.endsWith("/v1/admin/auth/login")) {
          return {
            status: 200,
            async json() {
              return {
                challenge: {
                  type: "new-password-required",
                  session:
                    "AYABeKFueu5U9joZP1ReowXft6cAHQABAAdTZXJ2aWNlABBDb2duaXRvVXNlclBvb2xzAAEAB2F3cy1rbXMAS2Fybjphd3M6a21zOnVzLXdlc3QtMjowMTU3MzY3MjcxOTg6a2V5LzI5OTFhNGE5LTM5YTAtNDQ0Mi04MWU4LWRkYjY4NTllMTg2MQC4AQIBAHhjxv5lVLhE2_WNrC1zuomqn08qDUUp3z9v4EGAjazZ-wE2c0wm9qfxE-BPAu8Gpx4MAAAAfjB8BgkqhkiG9w0BBwagbzBtAgEAMGgGCSqGSIb3DQEHATAeBglghkgBZQMEAS4wEQQMlLgg5D6C32sJPPy0AgEQgDu1evVNHUBRQ15lKivK_umnDVhIV4SB0ZQ3TeCjeYFyvkEZ8zTRjt_qf0lhKEk0a6v3hb-lgBTNmhOwawIAAAAADAAAEAAAAAAAAAAAAAAAAADgQciHS7Mgs07x_unRWSD5_____wAAAAEAAAAAAAAAAAAAAAEAAADV2LrCQXlk1MajCX1zgSqXnB_D7tkzuWNsqI43OB9-MZnk5RqQz_j_F85WBZOg6RMtC6_hPVUdVFMv6Qa2XNZKqh9ozWp4zPe4EQFb3X9hIMNuIP1imG2gP-zUgGbiYsCLJ2AWknq4q1s1YHWOMDYCaLeQgJt7awkPAvsP7zG2B5ktH8fcY6lUDb-jXEcziG4mvrjWlb1xel6kHuhgz6q0RxNVdhdfL1ZQ2-oTFexLcgUFBbpfu3vfvCYCCCDO9Z28p-sxn07qC6QrpCwRa8OP741GuJ26hst5EfrHITUykHeB72PwMw",
                  userId: "00000000-0000-0000-0000-000000000001",
                  email: "somebody@formant.example",
                },
              };
            },
          };
        }

        return {
          status: 404,
        };
      });

      expect(
        authStore.login("jim@starfleet.com", "jamestkirk")
      ).rejects.toThrowError();
    });

    it("should handle a 403 as a failure", async () => {
      const authStore = new AuthenticationStore({
        apiUrl: "https://vitest.invalid",
        addAccessTokenRefreshListener: vi.fn(),
        refreshAuthToken: vi.fn(),
      });

      mockFetch.mockImplementation((url) => {
        if (url.endsWith("/v1/admin/auth/login")) {
          return {
            status: 430,
            async json() {
              return {
                message: "Invalid username or password",
              };
            },
          };
        }

        return {
          status: 404,
        };
      });

      expect(
        authStore.login("jim@starfleet.com", "jamestkirk")
      ).rejects.toThrowError();
    });
  });

  describe("advanced", () => {
    it("should handle a successful login", async () => {
      const authStore = new AuthenticationStore({
        apiUrl: "https://vitest.invalid",
        addAccessTokenRefreshListener: vi.fn(),
        refreshAuthToken: vi.fn(),
      });

      mockFetch.mockImplementation((url) => {
        if (url.endsWith("/v1/admin/auth/login")) {
          return {
            status: 200,
            async json() {
              return {
                authentication: {
                  accessToken: fakeJwt({
                    sub: "00000000-0000-0000-0000-000000000002",
                  }),
                  refreshToken: fakeJwt({}),
                  organizationId: "00000000-0000-0000-0000-000000000001",
                  userId: "00000000-0000-0000-0000-000000000002",
                },
              };
            },
          };
        }

        if (
          url.endsWith("/v1/admin/users/00000000-0000-0000-0000-000000000002")
        ) {
          return {
            status: 200,
            async json() {
              return {};
            },
          };
        }

        return {
          status: 404,
        };
      });

      const actual: any = await authStore.login(
        "jim@starfleet.com",
        "jamestkirk",
        { advanced: true }
      );
      expect(actual.result).toBe("success");
      expect(actual.authentication).toMatchObject({
        accessToken:
          "eyJ0eXAiOiJKV1QiLCJhbGciOiJub25lIn0.eyJzdWIiOiIwMDAwMDAwMC0wMDAwLTAwMDAtMDAwMC0wMDAwMDAwMDAwMDIifQ.",
        organizationId: "00000000-0000-0000-0000-000000000001",
        refreshToken: "eyJ0eXAiOiJKV1QiLCJhbGciOiJub25lIn0.e30.",
        userId: "00000000-0000-0000-0000-000000000002",
      });
    });

    it("should handle a challenge", async () => {
      const authStore = new AuthenticationStore({
        apiUrl: "https://vitest.invalid",
        addAccessTokenRefreshListener: vi.fn(),
        refreshAuthToken: vi.fn(),
      });

      mockFetch.mockImplementation((url) => {
        if (url.endsWith("/v1/admin/auth/login")) {
          return {
            status: 200,
            async json() {
              return {
                challenge: {
                  type: "new-password-required",
                  session:
                    "AYABeKFueu5U9joZP1ReowXft6cAHQABAAdTZXJ2aWNlABBDb2duaXRvVXNlclBvb2xzAAEAB2F3cy1rbXMAS2Fybjphd3M6a21zOnVzLXdlc3QtMjowMTU3MzY3MjcxOTg6a2V5LzI5OTFhNGE5LTM5YTAtNDQ0Mi04MWU4LWRkYjY4NTllMTg2MQC4AQIBAHhjxv5lVLhE2_WNrC1zuomqn08qDUUp3z9v4EGAjazZ-wE2c0wm9qfxE-BPAu8Gpx4MAAAAfjB8BgkqhkiG9w0BBwagbzBtAgEAMGgGCSqGSIb3DQEHATAeBglghkgBZQMEAS4wEQQMlLgg5D6C32sJPPy0AgEQgDu1evVNHUBRQ15lKivK_umnDVhIV4SB0ZQ3TeCjeYFyvkEZ8zTRjt_qf0lhKEk0a6v3hb-lgBTNmhOwawIAAAAADAAAEAAAAAAAAAAAAAAAAADgQciHS7Mgs07x_unRWSD5_____wAAAAEAAAAAAAAAAAAAAAEAAADV2LrCQXlk1MajCX1zgSqXnB_D7tkzuWNsqI43OB9-MZnk5RqQz_j_F85WBZOg6RMtC6_hPVUdVFMv6Qa2XNZKqh9ozWp4zPe4EQFb3X9hIMNuIP1imG2gP-zUgGbiYsCLJ2AWknq4q1s1YHWOMDYCaLeQgJt7awkPAvsP7zG2B5ktH8fcY6lUDb-jXEcziG4mvrjWlb1xel6kHuhgz6q0RxNVdhdfL1ZQ2-oTFexLcgUFBbpfu3vfvCYCCCDO9Z28p-sxn07qC6QrpCwRa8OP741GuJ26hst5EfrHITUykHeB72PwMw",
                  userId: "00000000-0000-0000-0000-000000000001",
                  email: "somebody@formant.example",
                },
              };
            },
          };
        }

        return {
          status: 404,
        };
      });

      const actual: any = await authStore.login(
        "jim@starfleet.com",
        "jamestkirk",
        { advanced: true }
      );

      expect(actual.result).toBe("challenged");
      expect(actual.challenge).toMatchObject({
        type: "new-password-required",
        session:
          "AYABeKFueu5U9joZP1ReowXft6cAHQABAAdTZXJ2aWNlABBDb2duaXRvVXNlclBvb2xzAAEAB2F3cy1rbXMAS2Fybjphd3M6a21zOnVzLXdlc3QtMjowMTU3MzY3MjcxOTg6a2V5LzI5OTFhNGE5LTM5YTAtNDQ0Mi04MWU4LWRkYjY4NTllMTg2MQC4AQIBAHhjxv5lVLhE2_WNrC1zuomqn08qDUUp3z9v4EGAjazZ-wE2c0wm9qfxE-BPAu8Gpx4MAAAAfjB8BgkqhkiG9w0BBwagbzBtAgEAMGgGCSqGSIb3DQEHATAeBglghkgBZQMEAS4wEQQMlLgg5D6C32sJPPy0AgEQgDu1evVNHUBRQ15lKivK_umnDVhIV4SB0ZQ3TeCjeYFyvkEZ8zTRjt_qf0lhKEk0a6v3hb-lgBTNmhOwawIAAAAADAAAEAAAAAAAAAAAAAAAAADgQciHS7Mgs07x_unRWSD5_____wAAAAEAAAAAAAAAAAAAAAEAAADV2LrCQXlk1MajCX1zgSqXnB_D7tkzuWNsqI43OB9-MZnk5RqQz_j_F85WBZOg6RMtC6_hPVUdVFMv6Qa2XNZKqh9ozWp4zPe4EQFb3X9hIMNuIP1imG2gP-zUgGbiYsCLJ2AWknq4q1s1YHWOMDYCaLeQgJt7awkPAvsP7zG2B5ktH8fcY6lUDb-jXEcziG4mvrjWlb1xel6kHuhgz6q0RxNVdhdfL1ZQ2-oTFexLcgUFBbpfu3vfvCYCCCDO9Z28p-sxn07qC6QrpCwRa8OP741GuJ26hst5EfrHITUykHeB72PwMw",
        userId: "00000000-0000-0000-0000-000000000001",
        email: "somebody@formant.example",
      });
    });

    it("should handle failed credentials", async () => {
      const authStore = new AuthenticationStore({
        apiUrl: "https://vitest.invalid",
        addAccessTokenRefreshListener: vi.fn(),
        refreshAuthToken: vi.fn(),
      });

      mockFetch.mockImplementation((url) => {
        if (url.endsWith("/v1/admin/auth/login")) {
          return {
            status: 403,
            async json() {
              return {
                message: "Invalid username or password.",
              };
            },
          };
        }

        return {
          status: 404,
        };
      });

      const actual: any = await authStore.login(
        "jim@starfleet.com",
        "jamestkirk",
        { advanced: true }
      );

      expect(actual.result).toBe("failure");
      expect(actual.reason).toBe("Invalid username or password.");
    });
  });
});
