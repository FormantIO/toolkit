# Formant Web SDK

A collection of utilities to help make custom web and 3D experiences on top of Formant using [ThreeJS](https://threejs.org/). Using these SDKs you can get information about your fleed of devices, create realtime 3D visualizations of your robots, and more.

* Embed your own application in Formant
* Create your own application that uses Formant's APIs and RealtimeSDK for data
* Create WebView's for mobile applications that show 3D visualization of your fleet

# How does it work?

We understand you want complete control over your user experience, so Formant provides:

* Isolated 3D elements that you can import into your project for only the features you want
* An easy device data manager that can help get useful realtime data sources you need
* UI elements to make your application fit in to Formant's ecosystem

# How do I get data?

This depends on the type of app you're making

## I'm making an application within Formant

You don't need to do anything, the data manager will be able to figure out from url query strings what the authentication should be to access device data.

## I'm making an application outside of formant

Using user login or service accounts, you can use the data manager to get access to device data.

```javascript
import { DataManager } from "@formant/data-manager"

DataManager.login("sam@robot.xyz","passwordsecret")
```
