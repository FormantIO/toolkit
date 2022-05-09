build:
	yarn build
	cd dist &&  sed -i 's/"assets/"\.\/assets/' index.html

pub_demo: build
	git add -f dist
	git commit -m 'publish demo'
	git push origin master
yarn:
	rm yarn.lock || true
	rm -rf node_modules || true
	yarn
