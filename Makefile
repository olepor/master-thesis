# TODO - add synctex for pdf to editor code maneuvering

.Phony: all
all:
	mkdir -p build/sections # Necessary for succesful compilation
	latexmk -output-directory="build" -pdf

.Phony: file
file:
	latexmk -output-dir="build" -pdf # $1 add the command-line argument passed in!

.Phony: time
time:
	latexmk -output=dir="build" -time

.Phony: draft
draft:
	latexmk -d -output-dir="build"

.Phony: clean
clean:
	rm -rf build/*
