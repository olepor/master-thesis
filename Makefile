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

.Phony: section
section: src/sections/$(section).tex
	@echo "Building the section $(section)"
	@mkdir -p tests/$(section)/build
	@cp -r src/ tests/$(section)/
	@mv tests/$(section)/main.tex tests/$(section)/maintemplate.tex
	@tests/scripts/createIncludeOnly $(section) tests/$(section)/maintemplate.tex
	# @cd tests/$(section)/ && pdflatex -output-directory="build" -halt-on-error ./main.tex
	@cp {bibliography,zot}.bib ./tests/$(section)/
	@cd tests/$(section)/ && latexmk --output-directory=./build -cd -bibtex -pdf  ./main.tex
	@open tests/$(section)/build/main.pdf


section ?= "empty"
.Phony: clean-section
clean-section:
	@rm -rf tests/$(section)

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

.Phony: check
check:
	chktex src/main.tex src/sections/*.tex

pg: pg.tex
	@latexmk -pdf -silent $<
	@open pg.pdf

.Phony: taxonomy
taxonomy: setup
	@cd src && latexmk -output-directory="build" -pdf taxonomy.tex # $1 add the command-line argument passed in!
	@open src/build/taxonomy.pdf
