export default interface ModuleConfig {
  sections: Section[];
}
export interface Section {
  section: string;
  contents: SectionContent[];
}
export interface SectionContent {
  topic: string;
  minHz: number;
  type: string;
}
