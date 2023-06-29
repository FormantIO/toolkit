import { IStream } from "../model/IStream";
import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";

export async function getStreams(): Promise<IStream[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const response = await fetch(`${FORMANT_API_URL}/v1/admin/streams`, {
    method: "GET",
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  const streams = await response.json();
  return streams.items.filter(
    (_: { enabled: boolean }) => _.enabled
  ) as IStream[];
}
