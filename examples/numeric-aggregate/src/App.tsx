import { Icon } from "@formant/ui-sdk";
import { NumericAggregateBar } from "./NumericAggregateBar";
import { Configuration } from "./Configuration";
import { useState, useReducer, useMemo } from "react";
import "./App.css";
import {
  IAggregateConfiguration,
  AggregatePeriod,
  AggregateType,
  IActions,
  ACTIONS,
} from "./types";

import { LoadingIndicator } from "./LoadingIndicator";
import { useConfiguration } from "./useConfiguration";
import { KeyValue, App as FormantApp } from "@formant/data-sdk";

const reducer = (
  state: IAggregateConfiguration,
  action: IActions
): IAggregateConfiguration => {
  const { type, payload } = action;
  const { value } = payload;
  switch (type) {
    case ACTIONS.SET_NUMBER:
      return { ...state, numAggregates: value as number };
    case ACTIONS.SET_TYPE:
      return { ...state, aggregateType: value as AggregateType };
    case ACTIONS.SET_PERIOD:
      return { ...state, aggregateBy: value as AggregatePeriod };
    case ACTIONS.SET_STREAM_NAME:
      return { ...state, streamName: value as string };
    case ACTIONS.SET_STREAM_KEY:
      return { ...state, numericSetKey: value as string };
    case ACTIONS.LOAD_CONFIGURATION:
      return value as IAggregateConfiguration;
    default:
      return state;
  }
};

const initialState: IAggregateConfiguration = {
  aggregateType: "sum",
  streamName: "$.host.disk",
  numericSetKey: "utilization",
  aggregateBy: "day",
  numAggregates: 5,
};

function App() {
  const [state, dispatch] = useReducer(reducer, initialState);
  const isLoading = useConfiguration((_: IAggregateConfiguration) =>
    dispatch({ type: ACTIONS.LOAD_CONFIGURATION, payload: { value: _ } })
  );
  const [show, setShow] = useState(false);

  const context = useMemo(() => {
    return FormantApp.getCurrentModuleContext();
  }, []);

  return (
    <div className="App">
      {isLoading ? (
        <LoadingIndicator />
      ) : (
        <>
          {!show ? (
            <div className="gear" onClick={() => setShow((prev) => !prev)}>
              <Icon name="settings" />
            </div>
          ) : (
            <div
              onClick={() => {
                KeyValue.set(context!, JSON.stringify(state));
                setShow((prev) => !prev);
              }}
              className="check"
            >
              <Icon name="check" />
            </div>
          )}
          {show ? (
            <Configuration
              state={state}
              setNumberAgg={(_: number) =>
                dispatch({ type: ACTIONS.SET_NUMBER, payload: { value: _ } })
              }
              setType={(_: AggregateType) =>
                dispatch({ type: ACTIONS.SET_TYPE, payload: { value: _ } })
              }
              setPeriod={(_: AggregatePeriod) =>
                dispatch({ type: ACTIONS.SET_PERIOD, payload: { value: _ } })
              }
              setStreamName={(_: string) =>
                dispatch({
                  type: ACTIONS.SET_STREAM_NAME,
                  payload: { value: _ },
                })
              }
              setStreamKey={(_: string) =>
                dispatch({
                  type: ACTIONS.SET_STREAM_KEY,
                  payload: { value: _ },
                })
              }
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
        </>
      )}
    </div>
  );
}

export default App;
