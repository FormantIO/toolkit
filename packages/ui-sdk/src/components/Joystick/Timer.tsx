import { Component, useEffect } from "react";
import { setPromiseInterval } from "./utils";
import { IShutdownPromiseInterval, Timeout } from "./types";

interface IUseTimerOptions {
  enabled?: boolean;
  interval: number;
  wait?: boolean;
}

export function useTimer(callback: () => void, options: IUseTimerOptions) {
  const { enabled = true, interval, wait = false } = options;

  useEffect(() => {
    if (!enabled) {
      return;
    }

    if (wait) {
      const { stop } = setPromiseInterval({
        func: async () => callback(),
        delay: interval,
      });

      return () => stop();
    } else {
      const timeout = setInterval(callback, interval);
      const stop = () => {
        clearInterval(timeout);
      };

      return () => stop();
    }
  }, [callback, interval, wait, enabled]);
}

export interface ITimerProps {
  onTick: () => void;
  interval: number;
  wait?: boolean;
}

export class Timer extends Component<ITimerProps> {
  private interval?: Timeout;
  private promiseInterval?: IShutdownPromiseInterval;

  public componentDidMount() {
    const { onTick: callback, interval, wait } = this.props;
    if (wait) {
      this.promiseInterval = setPromiseInterval({
        func: async () => callback(),
        delay: interval,
      });
    } else {
      this.interval = setInterval(callback, interval);
    }
  }

  public async componentWillUnmount() {
    if (this.interval) {
      clearInterval(this.interval);
    }
    if (this.promiseInterval) {
      await this.promiseInterval.stop();
    }
  }

  public render() {
    return false;
  }
}
