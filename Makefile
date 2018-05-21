# TODO - add synctex for pdf to editor code maneuvering

.Phony: all
all: setup
	latexmk -output-directory="build" -pdf -silent
	open build/main.pdf

.Phony: setup
setup:
	mkdir -p build/sections # Necessary for succesful compilation

.Phony: file
file: setup
	latexmk -output-directory="build" -pdf # $1 add the command-line argument passed in!

.Phony: time
time:
	latexmk -output-directory="build" -time

.Phony: draft
draft: setup
	latexmk -d -output-directory="build"

.Phony: clean
clean:
	@rm -rf build/*
	@latexmk -C
	@latexmk -C pg.tex

.Phony: log
log:
	@less build/main.log

pg: pg.tex
	@latexmk -pdf -silent $<
	@open pg.pdf
