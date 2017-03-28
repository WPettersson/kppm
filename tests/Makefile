EXDIR = ../Examples

all: 2KP50 2AP05 3AP05 3KP10 4AP05 4KP10;

%:
	@echo -n "Testing $* "
	@$(TARGETDIR)/kppm -p $(EXDIR)/$** -o $*.out
	@diff -w -I'seconds\|solved\|Using' $(EXDIR)/$*.out $*.out && echo -n "t1 passed " && rm $*.out || echo -n "t1 failed "
	@$(TARGETDIR)/kppm -t 2 -s 2 -p $(EXDIR)/$**.lp -o $*.out
	@diff -w -I'seconds\|solved\|Using' $(EXDIR)/$*.out $*.out && echo -n "t2 passed " && rm $*.out || echo -n "t2 failed "
	@echo
