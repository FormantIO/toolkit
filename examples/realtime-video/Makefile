build:
	npm run build

clean:
	rm -rf node_modules package-lock.json dist

install:
	npm install

clean-install: clean install

local-dev:
	npm run dev

local-build:
	npm run build
	cp schema.json dist

local-preview: local-build
	npm run preview

deploy-prep:
	npm run build
	cp schema.json dist
	cd dist && sed -i'' -e 's/"\/assets/"\.\/assets/g' index.html

deploy: deploy-prep
	git add -f dist
	git commit -m 'publish demo'
	git push

deploy-with-upstream: deploy-prep
	git add -f dist
	git commit -m 'publish demo'
	git push --set-upstream origin $(shell git branch --show-current)
