import { SceneGraphElement } from "../src/main";
import * as uuid from "uuid";

export function createScene() {
  const sg: SceneGraphElement[] = [
    {
      id: uuid.v4(),
      editing: false,
      type: "video",
      name: "Video",
      deviceContext: "Abc",
      children: [],
      visible: true,
      position: { type: "hud", x: 0.2, y: 0.2 },
      scale: { x: 0.25, y: 0.25, z: 0.25 },
      fieldValues: {},
      data: {},
      dataSources: [
        {
          id: uuid.v4(),
          sourceType: "realtime",
          rosTopicName: "/joint_states",
          rosTopicType: "sensor_msgs/JointState",
        },
      ],
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
          type: "boolean",
          value: true,
        },
      },
      data: {},
    },
  ];
  return sg;
}
