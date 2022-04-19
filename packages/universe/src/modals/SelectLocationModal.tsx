import * as React from "react";
import { Component } from "react";
import { IUniverseData } from "../IUniverseData";
import { IStreamCurrentValue } from "../../../data-sdk/src/model/IStreamCurrentValue";
import { Typography, Button } from "@formant/ui-sdk";
import { Modal } from "../modals/Modal";

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
  items: IStreamCurrentValue<"location">[];
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
      items: await this.props.universeData.getLocations(),
    });
    if (this.state.items.length > 0) {
      this.setState({
        locationStreamName: this.state.items[0].streamName,
        relativeToLong: this.state.items[0].currentValue.longitude,
        relativeToLat: this.state.items[0].currentValue.latitude,
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

  onChangeLocationStream = (ev: React.ChangeEvent<HTMLSelectElement>) => {
    this.setState({
      locationStreamName: ev.target.value,
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
      <Modal>
        <Typography variant="h1">Select Location Posittioning</Typography>
        Select the location stream you'd like to use and it's relative long lat
        (this helps provide accuracy).
        <hr />
        Relative to Longitude
        <input
          type="number"
          value={this.state.relativeToLong}
          onChange={this.onChangeLong}
        />
        Relative to Latitude
        <input
          type="number"
          value={this.state.relativeToLat}
          onChange={this.onChangeLat}
        />
        {this.state.items && (
          <>
            <select
              value={this.state.locationStreamName}
              onChange={this.onChangeLocationStream}
            >
              {this.state.items.map((_) => (
                <option key={_.streamName} value={_.streamName}>
                  _.streamName
                </option>
              ))}
            </select>
          </>
        )}
        <div>
          <Button onClick={onCancel}>Cancel</Button>
          <Button
            onClick={this.onSelectLocation}
            disabled={!this.state.locationStreamName}
          >
            Select
          </Button>
        </div>
      </Modal>
    );
  }
}
