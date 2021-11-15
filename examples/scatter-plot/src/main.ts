import * as D3 from "d3";
import { streams } from "./streams";
import { timeRange } from "./timeRange";
import "./style.css";
import utils from "./utils";

//Data should be an object of Arrays
//Data Values will be normalize to use them in the chart

const colors = [
  "#88A87E",
  "#13AEBF",
  "#76A7DC",
  "#494DB3",
  "#7565AA",
  "#874991",
  "#B57486",
  "#6D5058",
  "#D4A07F",
  "#BAC4E2",
  "#FFFFF",
];

let height = window.innerHeight / 2;
let width = window.innerWidth / 2;

const scatterPlot = (data: Object) => {
  let margin = { left: 50, right: 50, top: 50, bottom: 50 };
  let innerWidth = width - margin.left - margin.right;
  let innerHeight = height - margin.top - margin.bottom;
  // First we need to find the highest value, and smallest value in our data

  const { maxValueInOurdata, minValueInOurData } =
    utils.getMaxandMinValuesInOurData(data);

  //Second we need to normalize our values

  const normalizeData = Object.keys(data).map((stream: string) =>
    utils.normalizeTwoDimensionalArray(
      [-1, 1],
      minValueInOurData,
      maxValueInOurdata,
      data[stream]
    )
  );

  // Set Up Chart

  const svg = D3.select("svg").attr("width", width).attr("height", height);

  //Set up scales

  const xScale = D3.scaleTime()
    .domain([timeRange[0], timeRange[timeRange.length - 1]])
    .nice()
    .range([0, innerWidth]);

  const yScale = D3.scaleLinear().domain([-1, 1]).range([innerHeight, 0]);

  //Set up Axis

  const xAxis = D3.axisBottom(xScale).ticks(10);
  const yAxis = D3.axisLeft(yScale).tickFormat(D3.format(".1f"));

  const topAxis = D3.axisBottom(xScale).tickFormat("");
  const rightAxis = D3.axisLeft(yScale).tickFormat("");

  //Set up grid

  const xAxisGrid = D3.axisBottom(xScale)
    .tickSize(-innerHeight)
    .tickFormat("")
    .ticks(10);
  const yAxisGrid = D3.axisLeft(yScale).tickSize(-innerWidth).tickFormat("");

  //Append grids to svg

  svg
    .append("g")
    .attr(
      "transform",
      `translate(${margin.left}, ${innerHeight + margin.top} )`
    )

    .call(xAxisGrid);
  svg
    .append("g")
    .attr("transform", `translate(${margin.left}, ${margin.top} )`)
    .call(yAxisGrid);

  //Append Axis to svg

  svg
    .append("g")
    .call(xAxis)
    .attr("transform", `translate(${margin.left}, ${innerHeight + margin.top})`)
    .attr("stroke-width", "1")
    .attr("stroke", "#BAC4E2")
    .attr("fill", "black");

  svg
    .append("g")
    .call(topAxis)
    .attr("transform", `translate(${margin.left}, ${margin.top})`)
    .attr("stroke-width", "1")
    .attr("fill", "black");

  svg
    .append("g")
    .call(yAxis)
    .attr("transform", `translate(${margin.left}, ${margin.top})`)
    .attr("stroke-width", "1")
    .attr("stroke", "#BAC4E2")
    .attr("fill", "black");
  svg
    .append("g")
    .call(rightAxis)
    .attr("transform", `translate(${margin.left + innerWidth}, ${margin.top})`)
    .attr("stroke-width", "1")
    .attr("fill", "black");

  //Plot data on chart

  Object.keys(normalizeData).forEach((stream: string, idx: number) => {
    svg
      .selectAll()
      .data(normalizeData[stream])
      .enter()
      .append("circle")
      .attr("transform", `translate(${margin.left}, ${margin.top})`)
      .attr("fill", colors[idx])
      .attr("cx", (d) => xScale(d[0]))
      .attr("cy", (d) => yScale(d[1]))
      .attr("r", 2.5);
  });
};

scatterPlot(streams);
