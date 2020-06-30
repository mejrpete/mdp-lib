#!/bin/bash
dir=$1
problem=$2

timeout 30m time ../testrddl.out ${dir} ${problem}
