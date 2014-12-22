#!/bin/sh
#VIDEO="e:\20120126091829.MTS"
#AIS="e:\0126.txt"
#SHIOJI="e:\Data_20120126_000000.log"

VIDEO="e:\20120307095142.MTS"
SHIOJI="e:\Data_20120307_085210.log"
AIS="e:\0307.txt"
TVIDEO=10
TAIS=35
TSHIOJI=35

channel img clr
channel campar maincp
filter vfile video -i -o clr maincp
filter dswin dbgw -i clr -o
fcmd video Int 1470.638297872340425531914893617 1476.8669756134619767530198283066 0 0
fcmd video rot 0 -0.13089969389957471826927680763665 -0.01
fcmd video pos 0 8.437 0
fcmd video open $VIDEO
fcmd dbgw open w
syn yes
cyc 0.04166667
go
