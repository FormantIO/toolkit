export default interface ModuleConfig {
  sections: Section[];
}
export interface Section {
  title: string;
  contents: SectionContent[];
}
export interface SectionContent {
  topic: string;
  minHz: number;
}
