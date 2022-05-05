import * as React from "react";
import { Component, useEffect } from "react";
import { TextField, DialogContentText, Stack, Select } from "@formant/ui-sdk";
import { IUniverseData } from "../../model/IUniverseData";
import { Modal } from "./Modal";
import { ILocation } from "../../../../data-sdk/src/model/ILocation";
import { fork } from "../../../../common/fork";

interface ISelectLocationModalProps {
  deviceContext: string;
  universeData: IUniverseData;
  onSelect: (
    streamName: string,
    relativeToLong: number,
    relativeToLat: number
  ) => void;
  onCancel: () => void;
}

export function SelectLocationModal(props: ISelectLocationModalProps) {
  const [relativeToLong, setRelativeToLong] = React.useState(0);
  const [relativeToLat, setRelativeToLat] = React.useState(0);
  const [locationStreamName, setLocationStreamName] = React.useState<
    string | undefined
  >(undefined);
  const [items, setItems] = React.useState<
    { streamName: string; location: ILocation }[]
  >([]);

  useEffect((): void => {
    fork(
      (async () => {
        setItems(
          await props.universeData.getLatestLocations(props.deviceContext)
        );
        if (items.length > 0) {
          setLocationStreamName(items[0].streamName);
          setRelativeToLong(items[0].location.longitude);
          setRelativeToLat(items[0].location.latitude);
        }
      })()
    );
  }, []);

  const onSelectLocation = () => {
    if (locationStreamName) {
      props.onSelect(locationStreamName, relativeToLong, relativeToLat);
    }
  };

  const onChangeLocationStream = (stream: string) => {
    const i = items.findIndex((_) => _.streamName === stream);
    setLocationStreamName(stream);
    setRelativeToLong(items[i].location.longitude);
    setRelativeToLat(items[i].location.latitude);
  };

  const onChangeLong = (ev: React.ChangeEvent<HTMLInputElement>) => {
    setRelativeToLong(Number(ev.target.value));
  };

  const onChangeLat = (ev: React.ChangeEvent<HTMLInputElement>) => {
    setRelativeToLat(Number(ev.target.value));
  };

  const { onCancel } = props;

  return (
    <Modal
      open
      title="Select Location Positioning"
      acceptText="Select"
      onAccept={onSelectLocation}
      acceptDisabled={!locationStreamName}
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
            value={relativeToLong}
            onChange={onChangeLong}
          />
        </div>
        <div>
          <TextField
            label="Relative to Latitude"
            type="number"
            value={relativeToLat}
            onChange={onChangeLat}
          />
        </div>
        {items && (
          <Select
            label="Location Stream"
            value={locationStreamName}
            onChange={onChangeLocationStream}
            items={items.map((_) => ({
              label: _.streamName,
              value: _.streamName,
            }))}
          />
        )}
      </Stack>
    </Modal>
  );
}
