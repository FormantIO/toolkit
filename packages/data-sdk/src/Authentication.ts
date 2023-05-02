import { FORMANT_API_URL } from "./config";
import { App } from "./App";
import { IAuthenticationStore } from "./stores/IAuthenticationStore";
import { AuthenticationStore } from "./stores/AuthenticationStore";

export const Authentication: IAuthenticationStore = new AuthenticationStore({
  apiUrl: FORMANT_API_URL,
  refreshAuthToken: App.refreshAuthToken,
  addAccessTokenRefreshListener: App.addAccessTokenRefreshListener,
});
