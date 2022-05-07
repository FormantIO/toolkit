prepublish:
	yarn lint
	yarn build
	yarn types
	mkdir -p dist/types/universe/three-utils/webxr
	cp three-utils/webxr/XRHandMeshModel.d.ts dist/types/universe/three-utils/webxr/

threeutils:
	cp node_modules/three/examples/jsm/webxr/VRButton.js three-utils/webxr/
	cp node_modules/@types/three/examples/jsm/webxr/VRButton.d.ts three-utils/webxr/
	cp node_modules/three/examples/jsm/controls/OrbitControls.js three-utils/controls/
	cp node_modules/@types/three/examples/jsm/controls/OrbitControls.d.ts three-utils/controls/
	cp node_modules/three/examples/jsm/controls/TransformControls.js three-utils/controls/
	cp node_modules/@types/three/examples/jsm/controls/TransformControls.d.ts three-utils/controls/
	cp node_modules/three/examples/jsm/loaders/SVGLoader.js three-utils/loaders/
	cp node_modules/@types/three/examples/jsm/loaders/SVGLoader.d.ts three-utils/loaders/
	cp node_modules/three/examples/jsm/loaders/TGALoader.js three-utils/loaders/
	cp node_modules/@types/three/examples/jsm/loaders/TGALoader.d.ts three-utils/loaders/
	cp node_modules/three/examples/jsm/loaders/ColladaLoader.js three-utils/loaders/
	cp node_modules/@types/three/examples/jsm/loaders/ColladaLoader.d.ts three-utils/loaders/
	cp node_modules/three/examples/jsm/loaders/DDSLoader.js three-utils/loaders/
	cp node_modules/@types/three/examples/jsm/loaders/DDSLoader.d.ts three-utils/loaders/
	cp node_modules/three/examples/jsm/loaders/DRACOLoader.js three-utils/loaders/
	cp node_modules/@types/three/examples/jsm/loaders/DRACOLoader.d.ts three-utils/loaders/
	cp node_modules/three/examples/jsm/loaders/GLTFLoader.js three-utils/loaders/
	cp node_modules/@types/three/examples/jsm/loaders/GLTFLoader.d.ts three-utils/loaders/
	cp node_modules/three/examples/jsm/loaders/KTX2Loader.js three-utils/loaders/
	cp node_modules/@types/three/examples/jsm/loaders/KTX2Loader.d.ts three-utils/loaders/
	cp node_modules/three/examples/jsm/loaders/FBXLoader.js three-utils/loaders/
	cp node_modules/@types/three/examples/jsm/loaders/FBXLoader.d.ts three-utils/loaders/
	cp node_modules/three/examples/jsm/lines/Line*.js three-utils/lines/
	cp node_modules/@types/three/examples/jsm/lines/Line* three-utils/lines/