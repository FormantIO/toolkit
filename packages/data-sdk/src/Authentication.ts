import { FORMANT_API_URL } from "./config";
import { IAuthenticationStore } from "./stores/IAuthenticationStore";
import { AuthenticationStore } from "./stores/AuthenticationStore";
import { refreshAuthToken } from "./message-bus/senders/refreshAuthToken";
import { addAccessTokenRefreshListener } from "./message-bus/listeners/addAccessTokenRefreshListener";

export const Authentication: IAuthenticationStore = new AuthenticationStore({
  apiUrl: FORMANT_API_URL,
  refreshAuthToken,
  addAccessTokenRefreshListener,
});
