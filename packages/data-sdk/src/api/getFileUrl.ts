import { FORMANT_API_URL } from "../config";
import { Authentication } from "../Authentication";

export async function getFileUrl(uuid: string) {
  const data = await fetch(`${FORMANT_API_URL}/v1/admin/files/query`, {
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
