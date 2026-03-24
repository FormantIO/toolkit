import { Device } from "../devices/Device";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";
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
  const response = await fetch(`${DataSdk.adminApi}/groups/${groupId}`, {
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
  });

  const { tagKey, tagValue } = await response.json();

  const devices = await queryDevices({
    tags: { [tagKey]: [tagValue] },
    enabled: true,
    type: "default",
  });

  return devices;
}
