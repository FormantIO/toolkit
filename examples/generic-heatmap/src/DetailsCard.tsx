import styles from "./App.module.scss";
import { FC, useEffect, useState } from "react";
import { Typography, useFormant } from "@formant/ui-sdk";
import { IConfiguration, HeatmapConfiguration } from "./types";

interface IDetailsCardPros {
  map: mapboxgl.Map | null;
}

export const DetailsCard: FC<IDetailsCardPros> = ({ map }) => {
  const context = useFormant();
  const config = context.configuration as HeatmapConfiguration;
  const [weight, setWeight] = useState(1);

  useEffect(() => {
    if (!map) return;
    const card = document.getElementById("card")!;
    setTimeout(() => {
      map.on("mouseover", "numeric-point", (e) => {
        const { x, y } = e.point;

        setWeight(1);
        if (config.numericStream) {
          const { weight } = e.features![0].properties!;
          setWeight(weight);
        }

        card.style.top = `${y}px`;
        card.style.left = `${x}px`;
        card.style.opacity = "1";
        card.style.zIndex = "2";
      });
      map.on("mouseout", "numeric-point", (event) => {
        card.style.opacity = "0";
        card.style.zIndex = "-1";
      });
    }, 6000);
  }, [map]);

  return (
    <div id="card" className={styles.card}>
      <Typography>{`${config.tooltipLabel ?? "Weight"}: ${weight}`}</Typography>
    </div>
  );
};
