build:
	npm run build


deploy:
	npm run build
	cd dist &&  sed -i'' -e 's/"\/assets/"\.\/assets/g' index.html 
	cp schema.json dist
	git add -f dist
	git commit -m 'publish demo'
	git push