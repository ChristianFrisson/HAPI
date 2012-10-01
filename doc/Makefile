
LATEX     =pdflatex
MAKEINDEX =makeindex
BIBTEX    =bibtex
FLAGS     =-interaction=nonstopmode --file-line-error-style

TARGET    = main


# Includes:
INCLUDES += introduction.tex

# Examples:
INCLUDES += examples/constant_force.tex
INCLUDES += examples/feedback_buffer_collector.tex
INCLUDES += examples/transfer_objects.tex
INCLUDES += examples/hapticsDeviceRegistration_h.tex
INCLUDES += examples/hapticsDeviceRegistration_cpp.tex
INCLUDES += examples/surface_example_h.tex
INCLUDES += examples/HapticForceField.tex
INCLUDES += examples/SimpleThreadPrint_cpp.tex
INCLUDES += examples/SimpleThreadPrintLock_cpp.tex
INCLUDES += examples/PeriodicThreadCallbacks_cpp.tex


# Compile once
fast:	$(GRAPHS) $(INCLUDES)
	$(LATEX) $(FLAGS) $(TARGET)

# Compile with bibtex
bib:	$(GRAPHS) $(INCLUDES)
	$(LATEX) $(FLAGS) $(TARGET)
	$(BIBTEX) $(TARGET)
	$(LATEX) $(FLAGS) $(TARGET)
	$(LATEX) $(FLAGS) $(TARGET)

# Compile and set up indices
index:	$(GRAPHS) $(INCLUDES)
	$(LATEX) $(FLAGS) $(TARGET)
	$(MAKEINDEX) $(TARGET)
	$(LATEX) $(FLAGS) $(TARGET)
	$(LATEX) $(FLAGS) $(TARGET)

# Compile and set up bibtex and indices
bibindex:	$(GRAPHS) $(INCLUDES)
	$(LATEX) $(FLAGS) $(TARGET)
	$(MAKEINDEX) $(TARGET)
	$(BIBTEX) $(TARGET)
	$(LATEX) $(FLAGS) $(TARGET)
	$(LATEX) $(FLAGS) $(TARGET)

clean:
	rm -f *.pdf *.aux *.brf *.log *.out *.toc \
		*.idx *.ilg *.ind *.bbl *.blg examples/*.tex


# Rules to compile examples (x3d, cpp, h and hh)
# using highlight 2.4 (http://www.andre-simon.de)
#
# See example in examples. Use like this:
# \input{examples/example.tex}
#

HIGHLIGHT_OPTIONS = -Trbf -s edit-jedit --out-format=latex -c highlight_theme.tex

%.tex:	%.x3d
	cp $< $(<:%.x3d=%.xml)
	highlight $(HIGHLIGHT_OPTIONS) -i $(<:%.x3d=%.xml) -o $@
	rm $(<:%.x3d=%.xml)
	cat $@ | sed -e "s/\\\\par/\\\\newline/g" > $@.tmp
	mv $@.tmp $@

%.tex:	%.cpp
	highlight $(HIGHLIGHT_OPTIONS) -i $< -o $@

%.tex:	%.h
	highlight $(HIGHLIGHT_OPTIONS) -i $< -o $@

%.tex:	%.hh
	highlight $(HIGHLIGHT_OPTIONS) -i $< -o $@


# Rule to compile figures
#

%.pdf:	%.fig
	fig2dev -L pstex $(@:%.pdf=%.fig) > $(@:%.pdf=%.eps)
	fig2dev -L pstex_t -p $@ $(@:%.pdf=%.fig) > $(@:%.pdf=%.tex)
	epstopdf $(@:%.pdf=%.eps)
	rm -f $(@:%.pdf=%.eps)


# Rule to compile graphs
#

%.pdf:	%.gnuplot %.log
	gnuplot $(@:%.pdf=%.gnuplot)
	epstopdf $(@:%.pdf=%.eps)
	rm -f $(@:%.pdf=%.eps)

