import { Authentication } from "@formant/data-sdk";

export const sendCommand = async (
  device: any,
  name: string,
  topic: string,
  data?: string,
  time?: Date
): Promise<void> => {
  let commands = await device.getAvailableCommands();
  let command = commands.find((_: { name: string }) => _.name === name);

  if (!command) {
    await fetch("https://api-dev.formant.io/v1/admin/command-templates", {
      method: "POST",
      body: JSON.stringify({
        command: "formant.publish_ros_topic",
        enabled: true,
        name: name,
        parameterEnabled: true,
        parameterMeta: {},
        tags: {},
      }),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    commands = await device.getAvailableCommands();
    command = commands.find((_: { name: string }) => _.name === name);
  }

  let d: string;

  if (data === undefined) {
    if (command.parameterEnabled && command.parameterValue) {
      d = command.parameterValue;
    } else {
      throw new Error(
        "Command has no default parameter value, you must provide one"
      );
    }
  } else {
    d = data;
  }

  let parameter = {
    value: d,
    scrubberTime: (time || new Date()).toISOString(),
    meta: {
      topic,
    },
  };

  const com = await fetch(`https://api-dev.formant.io/v1/admin/commands`, {
    method: "POST",
    body: JSON.stringify({
      commandTemplateId: command.id,
      organizationId: device.organizationId,
      deviceId: device.id,
      command: command.command,
      parameter: parameter,
      userId: Authentication.currentUser?.id,
    }),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
};

export const getDevice = async (deviceId: string) => {
  if (await Authentication.waitTilAuthenticated()) {
    const response = await fetch(
      `https://api-dev.formant.io/v1/admin/devices/${deviceId}`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );

    const device = await response.json();
    return device.state.ros.topics;
  }
};
