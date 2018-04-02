# TODO - add synctex for pdf to editor code maneuvering

.Phony: all
all:
	latexmk -output-directory="build" -pdf

.Phony: file
file:
	latexmk -output-dir="build" -pdf # $1 add the command-line argument passed in!

.Phony: time
time:
	latexmk -output=dir="build" -time

.Phony: draft
draft:
	latexmk -d

.Phony: clean
clean:
	latexmk -c
