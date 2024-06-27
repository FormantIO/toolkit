prepublish:
	yarn build
	yarn types
	npx terser -- dist/formant-universe-connector.es.js > formant-universe-connector.es.js
	mv formant-universe-connector.es.js dist/formant-universe-connector.es.js
	npx terser -- dist/formant-universe-connector.umd.js > formant-universe-connector.umd.js
	mv formant-universe-connector.umd.js dist/formant-universe-connector.umd.js

