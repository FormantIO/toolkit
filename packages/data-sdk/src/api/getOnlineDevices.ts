import { Device } from "../devices/Device";
import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";
import { getDevices } from "./getDevices";

export async function getOnlineDevices(): Promise<Device[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${FORMANT_API_URL}/v1/queries/online-devices`, {
    method: "GET",
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  const devices = await data.json();
  const onlineIds = devices.items as string[];
  const allDevices = await getDevices();
  return allDevices.filter((_) => onlineIds.includes(_.id));
}
