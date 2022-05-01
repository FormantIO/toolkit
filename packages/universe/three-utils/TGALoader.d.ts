import { Texture, Loader, LoadingManager } from "three";

export class TGALoader extends Loader {
  constructor(manager?: LoadingManager);

  load(
    url: string,
    onLoad: (texture: Texture) => void,
    onProgress?: (event: ProgressEvent) => void,
    onError?: (event: ErrorEvent) => void
  ): void;
  loadAsync(
    url: string,
    onProgress?: (event: ProgressEvent) => void
  ): Promise<Texture>;
  parse(data: ArrayBuffer): Texture;
}
