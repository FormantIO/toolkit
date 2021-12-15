import "./style.css";
import mapboxgl from "mapbox-gl";
import robotDirection from "./images/deviceOrientation.png";

let latitude = -102.130150794983;
let longitude = 32.065137575241096;
let radius = 10;
let waveRadius = 25;
let waveFrequency = 60;
let meterHeight = 48;
let maxAltitudeInFeet = 400;
let droneCardX: number;
let droneCardY: number;
let droneId: string | null = null;
let updateSourceSpeedInMilliSeconds = 2000;
const altitudeDisplay = document.getElementById("alt_val");
const altitudeMeterContainer = document.getElementById("alt_cont");
const deviceCard = document.getElementById("device__card__container");

//This function emulates a drone and return its geojson

const droneGeoJSON = () => {
  latitude = latitude + 0.0001;

  let currentLocation = {
    name: "location",
    type: "FeatureCollection",
    features: [
      {
        type: "Feature",
        properties: {
          altitude: "345",
          rotation: 90,
        },
        geometry: {
          type: "Point",
          coordinates: [latitude, longitude],
        },
      },
    ],
  };

  return currentLocation;
};

// Updates the location of the drone every two seconds

const updateSource = setInterval(() => {
  updateSource;
  const geojson = droneGeoJSON();
  map.getSource("location").setData(geojson);
}, updateSourceSpeedInMilliSeconds);

mapboxgl.accessToken =
  "pk.eyJ1IjoiYWxlbmpkZXYiLCJhIjoiY2t3NWt5ZmExMTcxMDJvbW5kdDR2eGs1diJ9.aYT7nc_i5rp2hY4dt3CLrw";
const map = new mapboxgl.Map({
  container: "map",
  style: "mapbox://styles/alenjdev/ckwcflbv8201814n10hxbqz6q",
  center: [-102.13015079498291, 32.065137575241096],
  zoom: 16,
});

map.on("load", async () => {
  map.loadImage(robotDirection, (error, image) => {
    if (error) throw error;
    map.addImage("robot-direction", image!);
  });

  const geojson = droneGeoJSON();

  map.addSource("location", {
    type: "geojson",
    data: geojson,
  });

  map.addLayer({
    id: "location-solid",
    type: "circle",
    source: "location",
    paint: {
      "circle-radius": 4,
      "circle-color": "#20A0FF",
      "circle-stroke-width": 1,
      "circle-stroke-color": "#fff",
    },
  });
  map.addLayer({
    id: "location-wave-one",
    type: "circle",
    source: "location",
    paint: {
      "circle-opacity": 0,
      "circle-stroke-width": 2,
      "circle-stroke-color": "#20A0FF",
      "circle-radius": radius,
      "circle-radius-transition": {
        duration: 0,
      },
    },
  });

  map.addLayer({
    id: "drone-direction",
    type: "symbol",
    source: "location",
    layout: {
      "icon-image": "robot-direction",
      "icon-rotate": ["get", "rotation"],
    },
  });
  setInterval(() => {
    map.setPaintProperty("location-wave-one", "circle-radius", radius);
    radius = ++radius % waveRadius;
  }, waveFrequency);
});

//Displays device card and altitude on hover

map.on("mouseover", "location-solid", (event: HTMLElement | any) => {
  const droneAltitude = event.features[0].properties.altitude;

  let altitudeMargin = (meterHeight * droneAltitude) / maxAltitudeInFeet;
  altitudeMargin = meterHeight - altitudeMargin;

  let { layerX, layerY } = event.originalEvent;

  droneCardX = layerX + 10;
  droneCardY = layerY + 10;

  if (event.features.length === 0) return;

  deviceCard!.style.top = `${droneCardY}px`;
  deviceCard!.style.left = `${droneCardX}px`;
  deviceCard!.style.display = "flex";
  altitudeMeterContainer!.style.display = "flex";
  altitudeMeterContainer!.style.marginTop = `${altitudeMargin}vh`;
  altitudeDisplay!.textContent = `${droneAltitude}ft`;

  if (droneId) {
    map.removeFeatureState({
      source: "location",
      id: droneId,
    });
  }

  droneId = event.features[0].id;
});

//Remove device card and altidude from screen

map.on("mouseleave", "location-solid", () => {
  if (droneId) {
    map.setFeatureState(
      {
        source: "location",
        id: droneId,
      },
      {
        hover: false,
      }
    );
  }

  droneId = null;
  setTimeout(() => {
    deviceCard!.style.display = "none";
    altitudeMeterContainer!.style.display = "none";
  }, 7000);
});
