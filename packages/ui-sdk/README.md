# UI SDK

A library of UI components from Formant

[demo](https://formantio.github.io/toolkit/examples/ui-sdk/dist/index.html)

# Other components

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
