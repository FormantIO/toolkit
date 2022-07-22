import {
  Button,
  DialogContentText,
  Select,
  Stack,
  TextField,
} from "@formant/ui-sdk";
import * as React from "react";
import { useEffect } from "react";
import { IUniverseData, UniverseDataSource } from  "@formant/universe-core";
import { LayerType } from "../../layers";
import { LayerRegistry, LayerSuggestion } from "../../layers/LayerRegistry";
import { LayerFields, LayerFieldUnion } from "../../model/LayerField";
import { Modal } from "./Modal";

interface IAddLayerModalProps {
  onAddLayer: (
    type: LayerType,
    dataSources?: UniverseDataSource[],
    fields?: LayerFields,
    name?: string,
    deviceContext?: string
  ) => void;
  onCancel: () => void;
  universeData: IUniverseData;
  deviceContext?: string;
}

export function AddLayerModal(props: IAddLayerModalProps) {
  const [selectedItem, setSelectedItem] = React.useState<LayerType>("data");
  const [currentName, setCurrentName] = React.useState<string>("");
  const [currentDeviceId, setCurrentDeviceId] = React.useState<
    string | undefined
  >(undefined);
  const [selectedSources, setSelectedSources] = React.useState<
    UniverseDataSource[] | undefined
  >(undefined);
  const [currentFields, setCurrentFields] = React.useState<LayerFields>({});
  const [deviceContexts, setDeviceContexts] = React.useState<
    {
      deviceName: string;
      deviceId: string;
    }[]
  >([]);
  const [layerSuggestions, setLayerSuggestions] = React.useState<{
    nonDataLayers: string[];
    dataLayers: LayerSuggestion[];
  }>({ nonDataLayers: [], dataLayers: [] });

  const { onCancel } = props;

  useEffect(() => {
    props.universeData.getDeviceContexts().then((d) => {
      setDeviceContexts(d);
      setCurrentDeviceId(d[0].deviceId);
    });

    LayerRegistry.getLayerSuggestions(
      props.universeData,
      props.deviceContext
    ).then((_) => {
      setLayerSuggestions(_);
    });
  }, []);

  const onChangeName = (ev: React.ChangeEvent<HTMLInputElement>) => {
    setCurrentName(ev.target.value);
  };

  const onSelect = (
    layerType: LayerType,
    dataSources?: UniverseDataSource[]
  ) => {
    setSelectedItem(layerType);
    setSelectedSources(dataSources);
    setCurrentFields(LayerRegistry.getFields(layerType, "create"));
    if (layerType === "data") {
      props.universeData.getDeviceContexts().then((d) => {
        setCurrentDeviceId(d[0].deviceId);
      });
    } else {
      setCurrentDeviceId(undefined);
    }
  };

  const onChangeCurrentDeviceId = (device: string) => {
    setCurrentDeviceId(device);
  };

  const addItemToScene = () => {
    props.onAddLayer(
      selectedItem,
      selectedSources,
      currentFields,
      currentName,
      selectedItem === "data" ? currentDeviceId : props.deviceContext
    );
  };

  return (
    <Modal
      open
      title="Add Layer"
      acceptText="Add"
      onAccept={addItemToScene}
      onClose={onCancel}
    >
      <Stack spacing={2}>
        <DialogContentText>Add a 3D layer to your universe</DialogContentText>
        <TextField
          value={currentName}
          onChange={onChangeName}
          label="Layer Name"
          placeholder="My Layer"
        />
        <div>
          {layerSuggestions.nonDataLayers.map((layerType) => (
            <Button
              variant={selectedItem === layerType ? "contained" : "outlined"}
              key={`nondata-${layerType}`}
              onClick={onSelect.bind(undefined, layerType, undefined)}
              sx={{ m: 1 }}
            >
              {LayerRegistry.getCommonName(layerType)}
            </Button>
          ))}
        </div>
        {layerSuggestions.dataLayers.length > 0 && (
          <>
            <DialogContentText>Data derived 3D layers:</DialogContentText>
            <div>
              {layerSuggestions.dataLayers.map((suggestion) => {
                let name: string;

                if (suggestion.sources[0].sourceType === "realtime") {
                  name = suggestion.sources[0].rosTopicName;
                } else if (suggestion.sources[0].sourceType === "hardware") {
                  name = suggestion.sources[0].rtcStreamName;
                } else {
                  name = suggestion.sources[0].streamName;
                }

                return (
                  <Button
                    variant={
                      selectedItem === suggestion.layerType
                        ? "contained"
                        : "outlined"
                    }
                    key={`data-${suggestion.layerType}-${suggestion.sources[0].id}`}
                    sx={{ m: 1 }}
                    onClick={onSelect.bind(
                      undefined,
                      suggestion.layerType,
                      suggestion.sources
                    )}
                  >
                    {LayerRegistry.getCommonName(suggestion.layerType)} [{name}]
                  </Button>
                );
              })}
            </div>
          </>
        )}
        {Object.entries(currentFields).map(([key, value]) => (
          <React.Fragment key={key}>
            {value.description}
            :
            <TextField
              value={value.value || ""}
              placeholder={value.placeholder.toString()}
              onChange={(ev) => {
                setCurrentFields({
                  ...currentFields,
                  [key]: {
                    ...currentFields[key],
                    value: ev.target.value,
                  } as LayerFieldUnion,
                });
              }}
            />
          </React.Fragment>
        ))}
        {selectedItem === "data" && currentDeviceId && (
          <>
            <DialogContentText>
              Select a device you'd like to use a source of data:
            </DialogContentText>
            <Select
              label="Device"
              value={currentDeviceId}
              onChange={onChangeCurrentDeviceId}
              items={deviceContexts.map((deviceContext) => ({
                label: deviceContext.deviceName,
                value: deviceContext.deviceId,
              }))}
            />
          </>
        )}
      </Stack>
    </Modal>
  );
}
