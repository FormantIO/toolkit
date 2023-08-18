import { IRtcPeer } from "@formant/realtime-sdk";

import { Authentication } from "../Authentication";
import { defaultRtcClientPool } from "../AppRtcClientPools";

export async function getPeers(): Promise<IRtcPeer[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const rtcClient = defaultRtcClientPool.get();
  try {
    return await rtcClient.getPeers();
  } finally {
    await rtcClient.shutdown();
  }
}
