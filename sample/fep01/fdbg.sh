#!/bin/sh

if [ $# -lt 1 ]; then
    echo "fdbg.sh <command> [<arg1> [<arg2]]"
    exit
fi

fset fep01 cmd $1

if [ $# -gt 1 ]; then
    fset fep01 iarg1 $2    
fi

if [ $# -gt 2 ]; then
    fset fep01 iarg2 $3
fi

fset fep01 push_cmd yes
