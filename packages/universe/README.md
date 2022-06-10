# Formant Universe

A toolkit for building 3D worlds with Formant.  Fleets of data are brimming with spatial data.  This is a library and editor that can help you put together 3D visualizations of robots on mobile, desktop, and VR.  It's meant to be an easy way for you to connect layers of information to data sources behind a single interface.

![image](https://user-images.githubusercontent.com/66638393/172955268-0fbc4fd4-1c4b-45d3-ae1e-bf28e470dd42.png)

# Mental model of this library

## What is universe?

A universe is a collection of layers

## What is a layer?

A layer is a collection of 3D element that can be driven by a data source and configured with fields.

## What is a layer field?

A layer field is a named property of a layer with a text, number, or boolean variable.  They can be configured with meta data to describe themselves in the UI dynamically.

## What is a layer part?

A layer part is a 3D element that a layer wants the world outside of itself to be aware of.  The might be important components other layers may want to test intersection interactions with for instance.

## What is a data source?

A data source is a representation of a specific source of information that might be present within a robot (example: A ROS message stream, a realtime hardware video source, a stream of JSON telemetry).

# Built-in Layers

## Data Layer
## Transform Tree Layer
## URDF Layer
## Geometry Layer
## Gltf Layer
## Ground Layer
## Image Layer
## Label Layer
## Point Cloud Layer
## Teleport Layer
## Video Layer
