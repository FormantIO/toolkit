build:
	npm run build

deploy: build
	cd dist &&  sed -i'' -e 's/"\/assets/"\.\/assets/g' index.html 
	cp schema.json dist
	./deploy.sh
	git add -f dist
	git commit -m 'publish demo'
	git push