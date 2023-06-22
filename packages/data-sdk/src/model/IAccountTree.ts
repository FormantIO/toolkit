import { IAccount } from "./IAccount";

export interface IAccountTree extends IAccount {
  children?: IAccountTree[];
}
