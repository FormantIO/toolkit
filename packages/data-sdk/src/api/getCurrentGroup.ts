import { Device } from "../devices/Device";
import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";
import { queryDevices } from "./queryDevices";

export async function getCurrentGroup(): Promise<Device[] | undefined> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  let urlParams = new URLSearchParams("");

  if (typeof window !== "undefined" && window.location) {
    urlParams = new URLSearchParams(window.location.search);
  }

  const groupId = urlParams.get("group");

  if (groupId === null || groupId.trim() === "") {
    return undefined;
  }
  const response = await fetch(
    `${FORMANT_API_URL}/v1/admin/groups/` + groupId,
    {
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    }
  );

  const { tagKey, tagValue } = await response.json();

  const devices = await queryDevices({
    tags: { [tagKey]: [tagValue] },
    enabled: true,
    type: "default",
  });

  return devices;
}
