import * as React from "react";
import { Authentication, Fleet } from "@formant/data-sdk";

// markup
const IndexPage = () => {
  start();
  return <div>hello!</div>;
};

async function start() {
  await Authentication.login("USERNAME", "SECRET");
  console.log(await Fleet.getDevices());
}

export default IndexPage;
