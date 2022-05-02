import { SceneGraphElement } from "../src/main";
import * as uuid from "uuid";

export function createScene() {
  const sg: SceneGraphElement[] = [
    {
      id: uuid.v4(),
      editing: false,
      type: "data",
      name: "Skynet-9000",
      deviceContext: "abc",
      children: [
        {
          id: uuid.v4(),
          editing: false,
          type: "label",
          name: "Name Label",
          deviceContext: "abc",
          children: [],
          visible: true,
          position: { type: "manual", x: 1, y: 0, z: 0 },
          fieldValues: {
            label_text: {
              type: "text",
              value: "Skynet-9000",
            },
          },
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
      fieldValues: {},
      data: {},
    },
  ];
  return sg;
}
