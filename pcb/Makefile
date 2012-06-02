PCB=pcb
FILEBASE=temp2morse
BOMFILE=${FILEBASE}.bom.txt
XYFILE=${FILEBASE}.xy.txt

default: gerber

gerber:
	${PCB} -x gerber --gerberfile ${FILEBASE} ${FILEBASE}.pcb
	mv *.gbr *.cnc ./phone-gerber

bom:
	gnetlist -g partslist3 -o temp2morse.bom temp2morse.sch

pcb:
	gsch2pcb --use-files --elements-dir ~/footprints temp2morse.sch

pdf: temp2morse.pdf

%.pdf: %.ps
	ps2pdf $< $@

%.ps: %.sch
	gschem -p -o $@ -s /usr/share/gEDA/scheme/print.scm $<
# gschem -p -o temp2morse.ps -s /usr/share/gEDA/scheme/print.scm temp2morse.sch

%.epsi: %.ps
	ps2epsi $<

# ps --(ps2epsi)--> epsi --(eps2png)--> png
eps: temp2morse.epsi
	# eps2png -jpg -mono -width 1200 $<
	eps2png -jpg -width 1200 $<

clean:
	rm -f *.gbr *.bom *.cnc *.pdf *.epsi *.png *.jpg *.gif ${BOMFILE} ${XYFILE}

