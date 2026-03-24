import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function getFileUrl(uuid: string) {
  const data = await fetch(`${DataSdk.adminApi}/files/query`, {
    method: "POST",
    body: JSON.stringify({
      fileIds: [uuid],
    }),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  const result = await data.json();
  if (result.fileUrls.length === 0) {
    throw new Error("File not found");
  }
  return result.fileUrls[0] as string;
}
