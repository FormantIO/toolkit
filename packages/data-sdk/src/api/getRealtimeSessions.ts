import { Authentication } from "../Authentication";
import { defaultRtcClientPool } from "../AppRtcClientPools";

export async function getRealtimeSessions(): Promise<{
  [key in string]: string[];
}> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const rtcClient = defaultRtcClientPool.get();
  try {
    return await rtcClient.getSessions();
  } finally {
    await rtcClient.shutdown();
  }
}
