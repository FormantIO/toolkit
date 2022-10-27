import { FormantProvider, Select, Icon } from "@formant/ui-sdk";
import { NumericAggregateBar } from "./NumericAggregateBar";
import { Configuration } from "./Configuration";
import { useState, useReducer } from "react";
import "./App.css";
import {
  IAggregateConfiguration,
  AggregatePeriod,
  AggregateType,
} from "./types";

enum ACTIONS {
  SET_NUMBER,
  SET_TYPE,
  SET_PERIOD,
  SET_STREAM_NAME,
  SET_STREAM_KEY,
}

interface IActions {
  type: ACTIONS;
  payload: {
    value: string | number | AggregatePeriod | AggregateType;
  };
}

const reducer = (
  state: IAggregateConfiguration,
  action: IActions
): IAggregateConfiguration => {
  const { type, payload } = action;
  const { value } = payload;
  switch (type) {
    case ACTIONS.SET_NUMBER:
      state.numAggregates = value as number;
      return state;
    case ACTIONS.SET_TYPE:
      state.aggregateType = value as AggregateType;
      return state;
    case ACTIONS.SET_PERIOD:
      state.aggregateBy = value as AggregatePeriod;
      return state;
    case ACTIONS.SET_STREAM_NAME:
      state.streamName = value as string;
    case ACTIONS.SET_STREAM_KEY:
      state.numericSetKey = value as string;
    default:
      return state;
  }
};

const basicQuery: IAggregateConfiguration = {
  aggregateType: "sum",
  streamName: "$.host.disk",
  numericSetKey: "utilization",
  aggregateBy: "day",
  numAggregates: 3,
};

function App() {
  const [state, dispatch] = useReducer(reducer, basicQuery);

  // const [numberAgg, setNumberAgg] = useState(2);
  // const [type, setType] = useState("sum");
  // const [by, setBy] = useState("day");
  // const [streamName, setStreamName] = useState("$.host.disk");
  // const [streamKey, setStreamKey] = useState("utilization");
  const [show, setShow] = useState(false);

  return (
    <div className="App">
      <div className="gear" onClick={() => setShow((prev) => !prev)}>
        <Icon name="settings" />
      </div>
      {show ? (
        <Configuration
          numberAgg={numberAgg}
          setNumberAgg={(_: number) =>
            dispatch({ type: ACTIONS.SET_NUMBER, payload: { value: _ } })
          }
          type={type}
          setType={(_: any) => setType(_)}
          by={by}
          setBy={(_: any) => setBy(_)}
          streamName={streamName}
          setStreamName={(_: any) => setStreamName(_)}
          streamKey={streamKey}
          setStreamKey={(_: any) => setStreamKey(_)}
        />
      ) : (
        <NumericAggregateBar
          aggregateType={state.aggregateType}
          streamName={state.streamName}
          numericSetKey={state.numericSetKey}
          aggregateBy={state.aggregateBy}
          numAggregates={state.numAggregates}
        />
      )}
    </div>
  );
}

export default App;
