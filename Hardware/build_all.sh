#!/bin/sh

report_board()
{
	PROJ=$1
	NUM_COLUMNS=$2

	echo $PROJ
	echo

	grep -B 1 -A $NUM_COLUMNS "; Flow Summary" $PROJ/*.flow.rpt
	grep -B 1 -A 5 "; Slow .* 85C Model Fmax Summary" $PROJ/*.sta.rpt | sed "s/^.*.sta.rpt.//"

	echo
}


SAVE_PATH=$PATH
set -e

QV=13.1
export PATH=$SAVE_PATH:/opt/altera/$QV/quartus/bin
QUARTUS_NAME=q$QV

for BOARD in de0-nano de1-soc marsohod2; do
	PROJ="proj-$QUARTUS_NAME-$BOARD"

	./make_project.sh $BOARD $PROJ
	cd $PROJ
	# We have to use 64-bit Quartus for Cyclone V
	make QFLAGS="--64bit" mips32r1-soc.svf
	cd ..

	cp $PROJ/mips32r1-soc.sof mips32r1-$QUARTUS_NAME-$BOARD.sof
	cp $PROJ/mips32r1-soc.svf mips32r1-$QUARTUS_NAME-$BOARD.svf
done

REPORT=REPORT.$QUARTUS_NAME
echo > $REPORT
report_board "proj-$QUARTUS_NAME-de0-nano" 19 >> $REPORT
report_board "proj-$QUARTUS_NAME-de1-soc" 22 >> $REPORT
report_board "proj-$QUARTUS_NAME-marsohod2" 19 >> $REPORT

QV=16.0
export PATH=$SAVE_PATH:/opt/altera/$QV/quartus/bin
QUARTUS_NAME=q$QV

for BOARD in de0-nano de1-soc marsohod3; do
	PROJ="proj-$QUARTUS_NAME-$BOARD"

	./make_project.sh $BOARD $PROJ
	cd $PROJ
	make mips32r1-soc.svf
	cd ..

	cp $PROJ/mips32r1-soc.sof mips32r1-$QUARTUS_NAME-$BOARD.sof
	cp $PROJ/mips32r1-soc.svf mips32r1-$QUARTUS_NAME-$BOARD.svf
done

REPORT=REPORT.$QUARTUS_NAME
echo > $REPORT
report_board "proj-$QUARTUS_NAME-de0-nano" 19 >> $REPORT
report_board "proj-$QUARTUS_NAME-de1-soc" 22 >> $REPORT
report_board "proj-$QUARTUS_NAME-marsohod3" 21 >> $REPORT
