#!/bin/bash
#Usage ./run_testsolver.sh [track] [k_reduced] [nsims] [model] [maxt] [extraflags]

track=$1
k_reduced=$2
nsims=$3
model=$4
maxt=$5
extraflags=$6
verbosity=$7

echo "--domain=racetrack --problem=../data/tracks/${track}.track --perror=.20 --pslip=0.40 --k=${k_reduced} --n=${nsims} --${model} --maxt=${maxt} --v=${verbosity} --mds=-1 ${extraflags}"

../../testreduced.out --domain=racetrack \
  --problem=../../data/tracks/${track}.track --perror=.20 --pslip=0.40 \
  --k=${k_reduced} --n=${nsims} --${model} --maxt=${maxt} \
  --v=${verbosity} --mds=-1 ${extraflags}
  