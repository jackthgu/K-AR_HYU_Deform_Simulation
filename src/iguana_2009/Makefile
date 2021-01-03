# This is a list of all files in the current directory ending in ".eps". The $(wildcard) is a globbing expression. This similar to how the shell expands *.eps
program_eps_SRCS := $(wildcard *.eps)
# This names all pdf files that we are going to build. It uses a substitution expression, which simply replaces ".eps" with ".pdf"
program_eps_PDFS := ${program_eps_SRCS:.eps=.pdf}

# see https://sites.google.com/site/michaelsafyan/software-engineering/how-to-write-a-makefile

%.pdf : %.eps 
	lua convertEPS.lua $^
all: long


long: iguana.tex $(program_eps_PDFS)
	pdflatex iguana.tex 
	bibtex iguana
	pdflatex iguana.tex 2>&1 >/dev/null
	bibtex iguana 2>&1 >/dev/null
	pdflatex iguana.tex 2>&1 >/dev/null
	pdflatex -synctex=1 iguana.tex 2>&1 >/dev/null
	cp -f iguana.pdf ~/Public

short: iguana.tex $(program_eps_PDFS)

	pdflatex -synctex=1 iguana.tex

clean:
	rm -f *.aux
	rm -f *.bbl
	rm -f *.lbl
	rm -f *.dvi
	rm -f *.brf
	rm -f *.blg
	rm -f *.bbl
	rm -f *.log
	rm -f *.pdfsync
	rm -f *~

latex: index.tex physics.bib $(program_eps_SRCS)

	latex index.tex
	bibtex index
	latex index.tex
	latex index.tex
	latex index.tex
	latex index.tex
	dvips -t a4 -Ppdf -G0 index.dvi
	ps2pdf -dPDFSETTINGS=/prepress -dCompatibilityLevel=1.3 -dAutoFilterColorImages=false -dAutoFilterGrayImages=false -dColorImageFilter=/FlateEncode -dGrayImageFilter=/FlateEncode -dMonoImageFilter=/FlateEncode -dDownsampleColorImages=false -dDownsampleGrayImages=false index.ps
	cp -f index.pdf paper1050.pdf


run:
	# how to set backward search in okula : configure -> editor -> custom text editor -> command: gvim --remote-silent +%l %f
	gvim -f index.tex &
	#kile index.tex &
	okular --unique index.pdf#src:380index.tex 1>/dev/null 2>&1 &

starte:
	gvim -f index.tex &
	evince index.pdf 1>/dev/null 2>&1 &
	
install_dependencies:
	sudo apt-get install texlive-latex-base texlive-latex-recommended texlive-humanities texlive-latex-extra texlive-fonts-recommended epstool
	sudo apt-get install ko.tex 
