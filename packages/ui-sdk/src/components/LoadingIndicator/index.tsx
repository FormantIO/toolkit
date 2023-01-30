import React from "react";
import loading from "../../images/loading.png";
import "./index.css";

export const LoadingIndicator = () => {
  return (
    <div className={"formant-loading-indicator"}>
      <img src={loading} />
    </div>
  );
};
