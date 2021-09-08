build:
	cd packages/data-manager && yarn run build && yarn run types && yarn run docs
	cd packages/3d-sdk-urdf && yarn run build && yarn run types
