import {
  createTeleopView,
  deleteTeleopView,
  fetchTeleopViews,
  getTeleopView,
  updateTeleopView,
} from "./api/teleopViews";

export class Views {
  constructor() {}
  static createTeleopView = createTeleopView;
  static updateTeleopView = updateTeleopView;
  static deleteTeleopView = deleteTeleopView;
  static getTeleopView = getTeleopView;
  static fetchTeleopViews = fetchTeleopViews;
}
