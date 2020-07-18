all:
	cat README.md

clean:
	rm -rf duckuments-dist

build:
	docker run -it  -v $(PWD):/pwd:delegated --workdir /pwd duckietown/docs-build:daffy hello

