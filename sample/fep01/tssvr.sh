#!/bin/sh

if [ $# -ne 1 ]; then
    echo "tssvr.sh <client address>"
    exit 1;
fi

# Configured as time sync server.
# Server should be instantiated first.
fset fep01 tcl $1
fset fep01 st tsv
