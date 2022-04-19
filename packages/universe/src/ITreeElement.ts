export interface TreeElement {
  title: string;
  textColor?: string;
  icons?: { icon: string; description: string; color?: string }[];
  children?: TreeElement[];
}

export type TreePath = number[];
