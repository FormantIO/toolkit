import { Button, Typography } from "@formant/ui-sdk";
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

  private onChangeCurrentDeviceId = (
    event: React.ChangeEvent<HTMLSelectElement>
  ) => {
    this.setState({
      currentDeviceId: event.target.value,
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
      <Modal>
        <Typography variant="h1">Add Layer</Typography>
        Add a 3D layer to your universe
        <input
          value={this.state.currentName}
          onChange={this.onChangeName}
          placeholder="Layer Name"
        />
        <div>
          {this.layerSuggestions.nonDataLayers.map((layerType) => (
            <Button
              key={"nondata-" + layerType}
              onClick={this.onSelect.bind(this, layerType, undefined)}
            >
              {LayerRegistry.getCommonName(layerType)}
            </Button>
          ))}
        </div>
        {this.layerSuggestions.dataLayers.length > 0 && (
          <>
            Data derived 3D layers:
            <div>
              {this.layerSuggestions.dataLayers.map((suggestion) => (
                <Button
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
                        ? "blue"
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
            <input
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
            <div>Select a device:</div>
            <select
              value={this.state.currentDeviceId}
              onChange={this.onChangeCurrentDeviceId}
            >
              {this.props.universeData
                .getDeviceContexts()
                .map((deviceContext) => (
                  <option
                    key={deviceContext.deviceId}
                    value={deviceContext.deviceId}
                  >
                    {deviceContext.deviceName}
                  </option>
                ))}
            </select>
          </>
        )}
        <hr />
        <div>
          <Button onClick={onCancel}>Cancel</Button>
          <Button onClick={this.addItemToScene}>Add</Button>
        </div>
      </Modal>
    );
  }
}
