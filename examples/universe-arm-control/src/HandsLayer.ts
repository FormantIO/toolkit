import { UniverseLayer } from "@formant/universe";

export class HandsLayer extends UniverseLayer {
  static layerTypeId = "hands";

  static commonName = "Hands";

  static description = "This is a scene that shows hand poses";

  init(): void {
    this.playSound(
      "https://formant-3d-models.s3.us-west-2.amazonaws.com/006_lifeWave2k.mp3",
      0.5,
      true
    );
  }
}
