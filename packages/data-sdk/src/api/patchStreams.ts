import { IStream } from "../model/IStream";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function patchStream(stream: IStream): Promise<IStream> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const response = await fetch(`${DataSdk.adminApi}/streams/${stream.id}`, {
    method: "PATCH",
    body: JSON.stringify(stream),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  return (await response.json()) as IStream;
}
