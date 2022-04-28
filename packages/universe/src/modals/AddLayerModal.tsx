import {
  Button,
  DialogContentText,
  Select,
  Stack,
  TextField,
} from "@formant/ui-sdk";
import * as React from "react";
import { Component } from "react";
import { equals } from "../../../common/equals";
import { IUniverseData, UniverseDataSource } from "../IUniverseData";
import { LayerType } from "../layers";
import { LayerRegistry, LayerSuggestion } from "../layers/LayerRegistry";
import { LayerFields } from "../layers/UniverseLayerContent";
import { Modal } from "../modals/Modal";

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
  layerSuggestions: {
    nonDataLayers: string[];
    dataLayers: LayerSuggestion[];
  };

  constructor(props: IAddLayerModalProps) {
    super(props);
    this.state = {
      selectedItem: "data",
      currentName: "",
      currentDeviceId: this.props.universeData.getDeviceContexts()[0].deviceId,
    };

    this.layerSuggestions = LayerRegistry.getLayerSuggestions(
      this.props.universeData,
      this.props.deviceContext
    );
  }

  private onChangeName = (ev: React.ChangeEvent<HTMLInputElement>) => {
    this.setState({
      currentName: ev.target.value,
    });
  };

  private addItemToScene = () => {
    this.props.onAddLayer(
      this.state.selectedItem,
      this.selectedSources,
      this.currentFields,
      this.state.currentName,
      this.state.currentDeviceId
    );
  };

  onSelect = (layerType: LayerType, dataSources?: UniverseDataSource[]) => {
    this.setState({
      selectedItem: layerType,
    });
    this.selectedSources = dataSources;
    this.currentFields = LayerRegistry.getFields(layerType);
    if (layerType === "data") {
      this.setState({
        currentDeviceId:
          this.props.universeData.getDeviceContexts()[0].deviceId,
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

  public render() {
    const { onCancel } = this.props;
    const { selectedItem } = this.state;

    return (
      <Modal
        open={true}
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
                key={"nondata-" + layerType}
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
                {this.layerSuggestions.dataLayers.map((suggestion) => (
                  <Button
                    variant="contained"
                    key={
                      "data-" +
                      suggestion.layerType +
                      "-" +
                      suggestion.sources[0].id
                    }
                    sx={{
                      backgroundColor:
                        selectedItem === suggestion.layerType &&
                        equals(this.selectedSources, suggestion.sources)
                          ? "#18d2ff"
                          : "white",
                    }}
                    onClick={this.onSelect.bind(
                      this,
                      suggestion.layerType,
                      suggestion.sources
                    )}
                  >
                    {LayerRegistry.getCommonName(suggestion.layerType)} [
                    {suggestion.sources[0].sourceType === "realtime"
                      ? suggestion.sources[0].rosTopicName
                      : suggestion.sources[0].sourceType === "hardware"
                      ? suggestion.sources[0].rtcStreamName
                      : suggestion.sources[0].streamName}
                    ]
                  </Button>
                ))}
              </div>
            </>
          )}
          {Object.entries(this.currentFields).map(([key, value]) => (
            <React.Fragment key={key}>
              {value.description}:
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
                items={this.props.universeData
                  .getDeviceContexts()
                  .map((deviceContext) => ({
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
