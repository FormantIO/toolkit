import { ITransform } from "../../../../data-sdk/src/model/ITransform";

export interface ITransformTreeNode {
  name: string;
  transform: ITransform;
  children?: ITransformTreeNode[];
}

export async function load(path: string): Promise<ITransformTreeNode> {
  const response = await fetch(path, { mode: "cors" });
  return response.json();
}
