import { SceneGraphElement } from "../src/main";
import * as uuid from "uuid";

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
      type: "point_cloud",
      name: "Point Cloud",
      deviceContext: "abc",
      children: [],
      visible: true,
      position: { type: "manual", x: 0, y: 1, z: 0 },
      fieldValues: {
        pointColor: {
          type: "number",
          value: 0xffffff,
          location: [],
        },
        pointSize: {
          type: "number",
          value: 0.075,
          location: [],
        },
        pointTexture: {
          type: "text",
          value:
            "https://formant-3d-models.s3.us-west-2.amazonaws.com/point.png",
          location: [],
        },
      },
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
          type: "boolean",
          location: [],
          value: true,
        },
      },
      data: {},
    },
    {
      children: [
        {
          id: uuid.v4(),
          editing: false,
          type: "test",
          name: "Test",
          deviceContext: undefined,
          children: [],
          visible: true,
          position: { type: "manual", x: 0, y: 0, z: 0 },
          fieldValues: {},
          data: {},
        },
      ],
      id: uuid.v4(),
      visible: true,
      editing: false,
      position: {
        type: "manual",
        x: 0,
        y: 0,
        z: 0,
      },
      fieldValues: {
        ghosted: {
          type: "text",
          location: [],
          value: "true",
        },
      },
      name: "Spot URDF",
      type: "device_visual_urdf",
      data: "A 3D model to represent a robot.",
      deviceContext: "ads",
      dataSources: [
        {
          id: uuid.v4(),
          sourceType: "realtime",
          rosTopicName: "/joint_states",
          rosTopicType: "sensor_msgs/JointState",
        },
      ],
    },
  ];
  return sg;
}
