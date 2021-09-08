build:
	cd data-manager && yarn run build && yarn run types && yarn run docs
	cd 3d-sdk-urdf && yarn run build && yarn run types
