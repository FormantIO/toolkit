import * as React from "react";
import { RecoilRoot } from "recoil";
import RecoilNexus from "recoil-nexus";
import { IUniverseAppProps, UniverseApp } from "./UniverseApp";

export function Universe({
  mode,
  universeData,
  initialSceneGraph,
  onSceneGraphChange,
  vr,
}: IUniverseAppProps) {
  return (
    <RecoilRoot>
      <RecoilNexus />
      <UniverseApp
        mode={mode}
        universeData={universeData}
        initialSceneGraph={initialSceneGraph}
        onSceneGraphChange={onSceneGraphChange}
        vr={vr}
      />
    </RecoilRoot>
  );
}
