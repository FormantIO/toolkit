build:
	cd packages/data-manager && yarn run build && yarn run types && yarn run docs
	cd packages/three-formant-urdf && yarn run build && yarn run types
