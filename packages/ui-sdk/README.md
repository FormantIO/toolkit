# UI SDK

A library of UI components from Formant.  These components are re-exported subset of [`mui`](https://mui.com).  You get the best of all worlds:
* components that look beautiful and fit in within a custom view/module.
* high quality industry standard component system for React (i.e. your knowledge is re-usable for other projects that aren't Formant related).
* unique formant specific components not offered by `mui`

[demo](https://formantio.github.io/toolkit/examples/ui-sdk/dist/index.html)

# Unique Formant components

* Timeline scrubber - COMING SOON

# Other MUI components

You can use the whole suite of components from [`mui`](https://mui.com) that will inherent our styles. The components that are explicitly tested from this packages have styling that is known to be of high quality.


# Developing on UI Sdk

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
