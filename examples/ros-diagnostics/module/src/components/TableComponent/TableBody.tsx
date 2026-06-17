import { Component, ReactNode } from "react";

interface ITableBodyProps {
  children: ReactNode;
}

export class TableBody extends Component<ITableBodyProps> {
  render(): ReactNode {
    return <tbody>{this.props.children}</tbody>;
  }
}
