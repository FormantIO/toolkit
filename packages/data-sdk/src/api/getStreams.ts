import { IStream } from "../model/IStream";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function getStreams(): Promise<IStream[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const response = await fetch(`${DataSdk.adminApi}/streams`, {
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
