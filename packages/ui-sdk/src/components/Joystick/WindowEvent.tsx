import { Component } from "react";

export interface IWindowEvent<T extends keyof WindowEventMap> {
  type: T;
  onEvent: (event: WindowEventMap[T]) => void;
  options?: boolean | AddEventListenerOptions;
}

export class WindowEvent<T extends keyof WindowEventMap> extends Component<
  IWindowEvent<T>
> {
  public componentDidMount() {
    const { type, options } = this.props;
    window.addEventListener(type, this.onEvent, options);
  }

  public componentWillUnmount() {
    const { type, options } = this.props;
    window.removeEventListener(type, this.onEvent, options);
  }

  public render() {
    return false;
  }

  private onEvent = (event: WindowEventMap[T]) => this.props.onEvent(event);
}
