import * as React from "react";
import { useEffect } from "react";

// markup
const IndexPage = () => {
  useEffect(() => {
    start();
  });

  return <div>hello!</div>;
};

async function start() {
  const { Authentication, Fleet } = require("@formant/data-sdk");
  await Authentication.login("USERNAME", "PASSWORD");
  console.log(await Fleet.getDevices());
}

export default IndexPage;
