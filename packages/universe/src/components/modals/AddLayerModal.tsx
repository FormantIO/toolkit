import {
  Button,
  DialogContentText,
  Select,
  Stack,
  TextField,
} from "@formant/ui-sdk";
import * as React from "react";
import { Component } from "react";
import { IUniverseData, UniverseDataSource } from "../../model/IUniverseData";
import { LayerType } from "../../layers";
import { LayerRegistry, LayerSuggestion } from "../../layers/LayerRegistry";
import { LayerFields } from "../../layers/UniverseLayerContent";
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

interface IAddLayerModalState {
  selectedItem: LayerType;

  currentName: string;

  currentDeviceId: string | undefined;
}

export class AddLayerModal extends Component<
  IAddLayerModalProps,
  IAddLayerModalState
> {
  private selectedSources?: UniverseDataSource[];

  private currentFields: LayerFields = {};

  private deviceContexts: {
    deviceName: string;
    deviceId: string;
  }[] = [];

  layerSuggestions: {
    nonDataLayers: string[];
    dataLayers: LayerSuggestion[];
  } = { nonDataLayers: [], dataLayers: [] };

  constructor(props: IAddLayerModalProps) {
    super(props);
    this.state = {
      selectedItem: "data",
      currentName: "",
      currentDeviceId: undefined,
    };
    this.props.universeData.getDeviceContexts().then((deviceContexts) => {
      this.deviceContexts = deviceContexts;
      this.setState({
        currentDeviceId: deviceContexts[0].deviceId,
      });
    });

    LayerRegistry.getLayerSuggestions(
      this.props.universeData,
      this.props.deviceContext
    ).then((layerSuggestions) => {
      this.layerSuggestions = layerSuggestions;
      this.forceUpdate();
    });
  }

  private onChangeName = (ev: React.ChangeEvent<HTMLInputElement>) => {
    this.setState({
      currentName: ev.target.value,
    });
  };

  onSelect = (layerType: LayerType, dataSources?: UniverseDataSource[]) => {
    this.setState({
      selectedItem: layerType,
    });
    this.selectedSources = dataSources;
    this.currentFields = LayerRegistry.getFields(layerType, "create");
    if (layerType === "data") {
      this.props.universeData.getDeviceContexts().then((deviceContexts) => {
        this.setState({
          currentDeviceId: deviceContexts[0].deviceId,
        });
      });
    } else {
      this.setState({
        currentDeviceId: undefined,
      });
    }
    this.forceUpdate();
  };

  private onChangeCurrentDeviceId = (device: string) => {
    this.setState({
      currentDeviceId: device,
    });
  };

  private onTextFieldChange(ev: React.ChangeEvent<HTMLInputElement>) {
    // @ts-ignore
    this.fields[this.key].value = ev.target.value;
    // @ts-ignore
    this.el.forceUpdate();
  }

  private addItemToScene = () => {
    this.props.onAddLayer(
      this.state.selectedItem,
      this.selectedSources,
      this.currentFields,
      this.state.currentName,
      this.state.selectedItem === "data"
        ? this.state.currentDeviceId
        : this.props.deviceContext
    );
  };

  public render() {
    const { onCancel } = this.props;
    const { selectedItem } = this.state;

    return (
      <Modal
        open
        title="Add Layer"
        acceptText="Add"
        onAccept={this.addItemToScene}
        onClose={onCancel}
      >
        <Stack spacing={2}>
          <DialogContentText>Add a 3D layer to your universe</DialogContentText>
          <TextField
            value={this.state.currentName}
            onChange={this.onChangeName}
            label="Layer Name"
            placeholder="My Layer"
          />
          <div>
            {this.layerSuggestions.nonDataLayers.map((layerType) => (
              <Button
                variant={selectedItem === layerType ? "contained" : "outlined"}
                key={`nondata-${layerType}`}
                onClick={this.onSelect.bind(this, layerType, undefined)}
                sx={{ m: 1 }}
              >
                {LayerRegistry.getCommonName(layerType)}
              </Button>
            ))}
          </div>
          {this.layerSuggestions.dataLayers.length > 0 && (
            <>
              <DialogContentText>Data derived 3D layers:</DialogContentText>
              <div>
                {this.layerSuggestions.dataLayers.map((suggestion) => {
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
                      onClick={this.onSelect.bind(
                        this,
                        suggestion.layerType,
                        suggestion.sources
                      )}
                    >
                      {LayerRegistry.getCommonName(suggestion.layerType)} [
                      {name}]
                    </Button>
                  );
                })}
              </div>
            </>
          )}
          {Object.entries(this.currentFields).map(([key, value]) => (
            <React.Fragment key={key}>
              {value.description}
              :
              <TextField
                value={value.value || ""}
                placeholder={value.placeholder}
                onChange={this.onTextFieldChange.bind({
                  fields: this.currentFields,
                  el: this,
                  key,
                })}
              />
            </React.Fragment>
          ))}
          {selectedItem === "data" && (
            <>
              <DialogContentText>
                Select a device you'd like to use a source of data:
              </DialogContentText>
              <Select
                label="Device"
                value={this.state.currentDeviceId}
                onChange={this.onChangeCurrentDeviceId}
                items={this.deviceContexts.map((deviceContext) => ({
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
}
