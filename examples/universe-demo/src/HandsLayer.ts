import { UniverseLayer, Hand, Label } from "@formant/universe";
import { Vector3 } from "three";

export class HandsLayer extends UniverseLayer {
  static layerTypeId = "hands";

  static commonName = "Hands";

  static description = "This is a scene that shows hand poses";

  labelLeft = new Label("");

  labelRight = new Label("");

  onHandsEnter(hands: Hand[]): void {
    hands[0].add(this.labelLeft);
    hands[1].add(this.labelRight);
  }

  onHandPosesChanged(hands: Hand[]): void {
    this.labelLeft.text = `${hands[0].getHandPose()}`;
    this.labelRight.text = `${hands[1].getHandPose()}`;
    if (hands[0].getHandPose() === "pinch") {
      this.playSound(
        "https://formant-3d-models.s3.us-west-2.amazonaws.com/levelup.wav",
        1,
        false,
        new Vector3(0, 2, 0)
      );
    }
    if (hands[1].getHandPose() === "pinch") {
      this.playSound(
        "https://formant-3d-models.s3.us-west-2.amazonaws.com/levelup.wav",
        1,
        false
      );
    }
  }

  onHandsMoved(hands: Hand[]): void {
    const wrist = hands[0].getJoint("wrist");
    if (wrist) {
      this.labelLeft.position.set(
        wrist.position.x,
        wrist.position.y,
        wrist.position.z
      );
    }
    const wrist2 = hands[1].getJoint("wrist");
    if (wrist2) {
      this.labelRight.position.set(
        wrist2.position.x,
        wrist2.position.y,
        wrist2.position.z
      );
    }
  }
}
