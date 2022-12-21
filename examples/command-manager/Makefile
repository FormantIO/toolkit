build:
	npm run build

deploy:
	cd dist &&  sed -i'' -e 's/"\/assets/"\.\/assets/g' index.html 
	git add -f dist
	git commit -m 'publish demo'
	git push