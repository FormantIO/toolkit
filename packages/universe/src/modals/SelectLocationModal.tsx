import * as React from "react";
import { Component } from "react";
import { TextField, DialogContentText, Stack, Select } from "@formant/ui-sdk";
import { IUniverseData } from "../IUniverseData";
import { Modal } from "./Modal";
import { ILocation } from "../../../data-sdk/src/model/ILocation";

interface ISelectLocationModalProps {
  universeData: IUniverseData;
  onSelect: (
    streamName: string,
    relativeToLong: number,
    relativeToLat: number
  ) => void;
  onCancel: () => void;
}

interface ISelectLocationModalState {
  relativeToLong: number;
  relativeToLat: number;
  locationStreamName: string | undefined;
  items: { streamName: string; location: ILocation }[];
}

export class SelectLocationModal extends Component<
  ISelectLocationModalProps,
  ISelectLocationModalState
> {
  constructor(props: ISelectLocationModalProps) {
    super(props);

    this.state = {
      relativeToLong: 0,
      relativeToLat: 0,
      locationStreamName: undefined,
      items: [],
    };
  }

  async componentDidMount() {
    this.setState({
      items: await this.props.universeData.getLatestLocations(),
    });
    if (this.state.items.length > 0) {
      this.setState({
        locationStreamName: this.state.items[0].streamName,
        relativeToLong: this.state.items[0].location.longitude,
        relativeToLat: this.state.items[0].location.latitude,
      });
    }
  }

  onSelectLocation = () => {
    if (this.state.locationStreamName) {
      this.props.onSelect(
        this.state.locationStreamName,
        this.state.relativeToLong,
        this.state.relativeToLat
      );
    }
  };

  onChangeLocationStream = (stream: string) => {
    this.setState({
      locationStreamName: stream,
    });
  };

  onChangeLong = (ev: React.ChangeEvent<HTMLInputElement>) => {
    this.setState({
      relativeToLong: parseFloat(ev.target.value) || 0,
    });
  };

  onChangeLat = (ev: React.ChangeEvent<HTMLInputElement>) => {
    this.setState({
      relativeToLat: parseFloat(ev.target.value) || 0,
    });
  };

  public render() {
    const { onCancel } = this.props;

    return (
      <Modal
        open
        title="Select Location Positioning"
        acceptText="Select"
        onAccept={this.onSelectLocation}
        acceptDisabled={!this.state.locationStreamName}
        onClose={onCancel}
      >
        <Stack spacing={2}>
          <DialogContentText>
            Select the location stream you would like to use and it's relative
            longitude and latitude:
          </DialogContentText>
          <div>
            <TextField
              label="Relative to Longitude"
              type="number"
              value={this.state.relativeToLong}
              onChange={this.onChangeLong}
            />
          </div>
          <div>
            <TextField
              label="Relative to Latitude"
              type="number"
              value={this.state.relativeToLat}
              onChange={this.onChangeLat}
            />
          </div>
          {this.state.items && (
            <Select
              label="Location Stream"
              value={this.state.locationStreamName}
              onChange={this.onChangeLocationStream}
              items={this.state.items.map((_) => ({
                label: _.streamName,
                value: _.streamName,
              }))}
            />
          )}
        </Stack>
      </Modal>
    );
  }
}
