import { SceneGraphElement } from "@formant/universe";
import * as uuid from "uuid";
import { ARM1_ID } from "./SimulatedUniverseData";

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
      type: "xbox",
      name: "Xbox",
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
      type: "hands",
      name: "Hands",
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
      name: "RoboArm1",
      deviceContext: ARM1_ID,
      children: [
        {
          id: uuid.v4(),
          editing: false,
          type: "label",
          name: "Name Label",
          deviceContext: ARM1_ID,
          children: [],
          visible: true,
          position: { type: "manual", x: 0, y: 0, z: 0 },
          fieldValues: {
            label_text: {
              location: [],
              type: "text",
              value: "RoboArm1",
            },
          },
          data: {},
        },
        {
          children: [],
          id: uuid.v4(),
          visible: true,
          editing: false,
          position: {
            type: "manual",
            x: 0,
            y: 0,
            z: 0,
          },
          fieldValues: {},
          name: "Arm URDF",
          type: "device_visual_urdf",
          data: "A 3D model to represent a robot.",
          dataSources: [
            {
              id: "89d29103-2686-4ae6-b525-f660dcd2e17a",
              sourceType: "realtime",
              rosTopicName: "armJoints",
              rosTopicType: "sensor_msgs/JointState",
            },
          ],
          deviceContext: ARM1_ID,
        },
      ],
      visible: true,
      position: {
        type: "manual",
        x: 0,
        y: 0,
        z: 0,
      },
      fieldValues: {},
      data: {},
    },
    {
      children: [],
      id: "ac2dc279-b50a-4c02-b65e-8c545be92b6c",
      visible: true,
      editing: true,
      position: {
        type: "manual",
        x: 0,
        y: 0,
        z: 0,
      },
      fieldValues: {
        url: {
          location: [],
          type: "text",
          value:
            "https://formant-3d-models.s3.us-west-2.amazonaws.com/tadao_ando.glb",
        },
      },
      name: "Grass",
      type: "3dmodel",
      data: "A 3D model.",
    },
  ];
  return sg;
}
