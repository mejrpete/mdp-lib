#!/bin/bash
dir=$1
problem=$2

timeout 10m time ../testrddl.out --problemdir=${dir} --instance=${problem}
