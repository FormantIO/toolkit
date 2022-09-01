import { Positioning } from "./SceneGraph";

export class PositioningBuilder {
  static fixed(x: number, y: number, z: number): Positioning {
    return { type: "manual", x, y, z };
  }

  static localization(stream: string): Positioning {
    return { type: "localization", stream };
  }

  static gps(
    stream: string,
    relativeLongLat: { long: number; lat: number }
  ): Positioning {
    return {
      type: "gps",
      stream,
      relativeToLatitude: relativeLongLat.lat,
      relativeToLongitude: relativeLongLat.long,
    };
  }

  static tranformTree(stream: string, end: string): Positioning {
    return {
      type: "transform tree",
      stream,
      end,
    };
  }

  static hud(x: number, y: number): Positioning {
    return {
      type: "hud",
      x,
      y,
    };
  }
}
