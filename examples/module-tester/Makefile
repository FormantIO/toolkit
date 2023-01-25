build:
	yarn build
pub_demo: yarn build
	git add -f dist
	git commit -m 'publish demo'
	git push origin master
yarn:
	rm yarn.lock || true
	rm -rf node_modules || true
	yarn
