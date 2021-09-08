build:
	cd packages/data-manager && yarn && yarn run build && yarn run types && yarn run docs
	cd packages/three-formant-urdf && yarn && yarn run build && yarn run types
