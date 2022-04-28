# UI SDK

A library of UI components from Formant. These components are re-exported subset of [`mui`](https://mui.com). You get the best of all worlds:

- components that look beautiful and fit in within a custom view/module.
- high quality industry standard component system for React (i.e. your knowledge is re-usable for other projects that aren't Formant related).
- unique formant specific components not offered by `mui`

[demo](https://formantio.github.io/toolkit/examples/ui-sdk/dist/index.html)

# Unique Formant components

- Timeline scrubber - COMING SOON

# Other MUI components

You can use the whole suite of components from [`mui`](https://mui.com) that will inherent our styles. The components that are explicitly tested from this packages have styling that is known to be of high quality.

# Setting up in your own project

```
yarn add @formant/ui-sdk
```

or check out our [ViteJS example](https://github.com/FormantIO/toolkit/tree/master/examples/ui-sdk)

# Font

Formant UI Sdk uses the [Inter](https://fonts.google.com/specimen/Inter) open source font, add this to your project

```html
<link rel="preconnect" href="https://fonts.googleapis.com" />
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin />
<link
  href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;700&display=swap"
  rel="stylesheet"
/>
```

# Formant React hooks

Within this project are a number of hooks that make access informationa about your fleet easy to do.

# Developing on `ui-sdk`

```
yarn
yarn dev
```

# Running storybook

```
yarn storybook
```

# Building for production

Weirdly, typescript requires npm layout package in order to generate its types

```
make
```

should run all that's necessary for creating a build
