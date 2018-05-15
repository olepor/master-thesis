# TODO - add synctex for pdf to editor code maneuvering

.Phony: all
all: setup bibfix
	latexmk -output-directory="build" -pdf
	open build/main.pdf

.Phony: bibfix
bibfix: zot.bib
	cat $< | sed -E "s/[_]\?{2,}//" | tee zot.bib

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
	rm -rf build/*
