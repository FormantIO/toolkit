import { useDevice } from "@formant/ui-sdk";
import { Universe, LayerRegistry } from "@formant/universe";
import { TelemetryUniverseData } from "@formant/universe-connector";
import { FC } from "react";
import { PositionLayer } from "./PositionLayer";

const telemetryData = new TelemetryUniverseData();

LayerRegistry.register(PositionLayer);

interface IProps {
  deviceId: string;
}

const UniverseApp: FC<IProps> = ({ deviceId }) => {
  return (
    <Universe
      initialSceneGraph={[
        {
          id: crypto.randomUUID(),
          editing: false,
          type: "ground",
          name: "Ground",
          deviceContext: deviceId,
          children: [],
          visible: true,
          position: { type: "manual", x: 0, y: 0, z: 0 },
          fieldValues: {
            flatAxes: {
              type: "boolean",
              value: true,
            },
          },
          data: {},
        },
        {
          id: crypto.randomUUID(),
          editing: false,
          type: "grid_map",
          name: "Localization",
          deviceContext: deviceId,
          children: [],
          visible: true,
          position: { type: "manual", x: 0, y: -0.5, z: 0 },
          fieldValues: {},
          data: {},
          dataSources: [
            {
              id: crypto.randomUUID(),
              sourceType: "telemetry",
              streamName: "map",
              streamType: "localization",
            },
          ],
        },
        {
          id: crypto.randomUUID(),
          editing: false,
          type: "PositionLayer",
          name: "Localization",
          deviceContext: deviceId,
          children: [],
          visible: true,
          position: { type: "manual", x: 0, y: -0.5, z: 0 },
          fieldValues: {},
          data: {},
          dataSources: [
            {
              id: crypto.randomUUID(),
              sourceType: "telemetry",
              streamName: "map",
              streamType: "localization",
            },
          ],
        },
      ]}
      universeData={telemetryData}
      mode="view"
      vr
    />
  );
};

export const App = () => {
  const device = useDevice();

  if (!device) return <></>;

  return <UniverseApp deviceId={device.id} />;
};
