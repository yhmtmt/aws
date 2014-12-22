#!/bin/sh
VIDEO="e:\20120126091829.MTS"
AIS="e:\0126.txt"
SHIOJI="e:\Data_20120126_000000.log"

#VIDEO="e:\20120307095142.MTS"
#SHIOJI="e:\Data_20120307_085210.log"
#AIS="e:\0307.txt"
TVIDEO=10
TAIS=35
TSHIOJI=35

channel img clr
channel img clr1
channel img clr2
channel pvt pvt
channel img gry
channel img gauss
channel img stbg
channel img stb
channel ais ais
channel ship ship
channel trck trckin
channel trck trckout
#channel ptz ptzin
#channel ptzc ptzout
#channel campar subcp
channel campar maincp
#filter hd5400 subcam -i ptzout -o subcam subcp ptzin
#fcmd subcam Int 1470.638297872340425531914893617 1476.8669756134619767530198283066 0 0
#fcmd subcam rot 0 -0.13089969389957471826927680763665 -0.01
#fcmd subcam pos 0 8.437 0
filter vfile video -i -o clr maincp
fcmd video Int 1470.638297872340425531914893617 1476.8669756134619767530198283066 0 0
fcmd video rot 0 -0.13089969389957471826927680763665 -0.01
fcmd video pos 0 8.437 0
fcmd video tsh $TVIDEO
filter nav navdat -i null -o pvt ais null
fcmd navdat tsh $TAIS
filter gry gry -i clr -o gry clr1
fcmd gry intvl 2
filter gauss gauss -i gry clr1 -o gauss clr2
fcmd gauss sigma 1.0
fcmd gauss intvl2
filter trck trck -i stbg trckin -o trckout
fcmd trck intvl 2
filter stab stb -i gauss clr2 -o stbg stb
fcmd stb itr 30
fcmd stb ipt bil 
fcmd stb wt rgd
fcmd stb alpha 0
fcmd stb alpha 2 1
fcmd stb beta 0.01
fcmd stb plv 2
fcmd stb roi 448 310 1024 64
fcmd stb disp
fcmd stb intvl 2
filter shioji shioji -i null -o ship
fcmd shioji tsh $TSHIOJI
filter dswin dbgw -i gauss -o
fcmd dbgw wmd w 1280 720
fcmd dbgw wmd f 1920 1080
fcmd dbgw open w
filter syswin sys -i stb maincp pvt ais ship trckout null null null -o trckin ptzout
#fcmd sys wmd w 1024 768
#fcmd sys wmd w 1920 1080
fcmd sys wmd w 1280 720
fcmd sys wmd f 1920 1080
fcmd sys own_mmsi 431100037
fcmd sys trck 32 32
#fcmd subcam open 192.244.155.125 admin:admin
############ open files #######
fcmd video open $VIDEO
#fcmd video open e:\20120307095142.MTS oneshot 
fcmd navdat fopen $AIS
fcmd shioji fopen $SHIOJI
fcmd sys open w
#fcmd sys disp
#sync no
syn yes
cyc 0.04166667
#cycle 0.1
#fcmd sys grid 60
#fcmd  stb through
go 9000
