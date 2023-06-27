# Formant Data SDK

A library for accessing the devices in your Formant fleet.

Check out the [project website](https://github.com/FormantIO/toolkit) for more information!

## Get Started

Install using `npm` or `yarn`:

```sh
$ npm install --save @formant/data-sdk@latest

# or with

$ yarn add @formant/data-sdk@latest
```

## License

Distributed under the MIT License. See LICENSE.txt for more information.

---

## Deploying a new version

```bash
# update changelog
vim CHANGELOG.md

# use npm to bump the version number
npm version X.Y.Z

# add all the changed files from the preversion/postversion tasks
git add .

# commit all the changes with a uniform commit message
git commit -m data-sdk@$(node -p "require('./package.json').version")

# add a unified annotated tag for this version
git tag -a release/data-sdk/$(node -p "require('./package.json').version") -m data-sdk@$(node -p "require('./package.json').version")

# publish this version to npm; use "--tag=next" for future releases
npm publish

# push everything to remote vcs with the tags
git push --follow-tags
```
