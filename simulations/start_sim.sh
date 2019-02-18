#!/bin/bash
echo "Iniciando Simulacoes"

i=10
cont=0
for run in {0..1400};
do
	echo "Run Number # $run"
	../Slicing -u Cmdenv -r $run -n ..:../../veins/examples/veins:../../veins/src/veins --image-path=../../veins/images -l ../../veins/src/veins --debug-on-errors=false omnetpp.ini > saida.txt
done
echo "Total de Simulacoes = $cont"