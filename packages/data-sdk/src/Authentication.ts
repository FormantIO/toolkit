import { addAccessTokenRefreshListener } from "./message-bus/listeners/addAccessTokenRefreshListener";
import { refreshAuthToken } from "./message-bus/senders/refreshAuthToken";
import { AuthenticationStore } from "./stores/AuthenticationStore";
import { IAuthenticationStore } from "./stores/IAuthenticationStore";

export const Authentication: IAuthenticationStore = new AuthenticationStore({
  refreshAuthToken,
  addAccessTokenRefreshListener,
});
