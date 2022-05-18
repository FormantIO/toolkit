import { HandheldController } from "../src/components/viewer/HandheldController";
import { Controller, UniverseLayer } from "../src/main";

export class XboxLayer extends UniverseLayer {
  static layerTypeId: string = "xbox";

  static commonName = "Xbox";

  static description = "Layer for xbox controlls.";

  static usesData = false;

  onHandheldControllerAxisChanged(
    _controller: HandheldController,
    _axis: number,
    _value: number
  ): void {
    console.log(_axis, _value);
  }

  onControllerButtonChanged(
    _controller: Controller,
    _button: number,
    _value: number
  ): void {
    if (_value === 1) {
      console.log(_button, _value);
      this.playSound(
        "https://formant-3d-models.s3.us-west-2.amazonaws.com/levelup.wav",
        1,
        false
      );
    }
  }
}
