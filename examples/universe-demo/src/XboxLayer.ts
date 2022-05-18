import { HandheldController, UniverseLayer } from "@formant/universe";

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

  onHandheldControllerButtonChanged(
    _controller: HandheldController,
    _button: number,
    _value: number
  ): void {
    console.log(_button, _value);
    _controller.vibrate({
      startDelay: 0,
      duration: 200,
      weakMagnitude: 1.0,
      strongMagnitude: 1.0,
    });
    debugger;
  }
}
