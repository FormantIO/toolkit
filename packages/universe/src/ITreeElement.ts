import { IconName } from "@formant/ui-sdk";

export interface TreeElement {
  title: string;
  textColor?: string;
  icons?: { icon: IconName; description: string; color?: string }[];
  children?: TreeElement[];
}

export type TreePath = number[];

export function treePathEquals(a: TreePath, b: TreePath) {
  if (a.length !== b.length) {
    return false;
  }
  return a.every((value, index) => value === b[index]);
}
