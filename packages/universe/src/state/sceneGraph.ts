import { atom } from "recoil";
import { SceneGraph } from "../model/SceneGraph";

export const sceneGraphAtom = atom<SceneGraph>({
  key: "sceneGraph",
  default: [],
  dangerouslyAllowMutability: true,
});
