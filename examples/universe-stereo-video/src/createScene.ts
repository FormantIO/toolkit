import { SceneGraphElement } from "@formant/universe";
import * as uuid from "uuid";
import { SPOT_ID } from "./SimulatedUniverseData";

export function createScene() {
  const sg: SceneGraphElement[] = [
    {
      id: uuid.v4(),
      editing: false,
      type: "teleport",
      name: "Teleport",
      deviceContext: undefined,
      children: [],
      visible: true,
      position: { type: "manual", x: 0, y: 0, z: 0 },
      fieldValues: {},
      data: {},
    },
    {
      id: uuid.v4(),
      editing: false,
      type: "data",
      name: "Spot-9000",
      deviceContext: SPOT_ID,
      children: [
        {
          id: uuid.v4(),
          editing: false,
          type: "video",
          name: "Video",
          deviceContext: "abcd",
          children: [],
          visible: true,
          position: { type: "manual", x: 0, y: 1.4, z: 0 },
          fieldValues: {
            videoShape: {
              type: "text",
              value: "stereo-side-by-side",
              location: [],
            },
          },
          dataSources: [
            {
              id: "89d29103-2686-4ae6-b525-f660dcd2e17a",
              sourceType: "realtime",
              rosTopicName: "armJoints",
              rosTopicType: "sensor_msgs/JointState",
            },
          ],
          data: {},
        },
      ],
      visible: true,
      position: { type: "manual", x: 0, y: 0, z: 0 },
      fieldValues: {},
      data: {},
    },
    {
      id: uuid.v4(),
      editing: false,
      type: "ground",
      name: "Ground",
      deviceContext: undefined,
      children: [],
      visible: true,
      position: { type: "manual", x: 0, y: 0, z: 0 },
      fieldValues: {
        flatAxes: {
          type: "text",
          location: [],
          value: "true",
        },
      },
      data: {},
    },
  ];
  return sg;
}
