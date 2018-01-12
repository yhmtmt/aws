#!/bin/sh

width=640
height=480

channel imgr img

filter test_vsrc vsrc -i -o
fset vsrc ch_img img
fset vsrc width $width height $height

filter gstenc enc -i -o
fset enc ch_in img
fset enc width $width height $height
fset enc fmt_in BGR8 fmt_out I420
fset enc fps 30
fset enc fppl /mnt/ssd1/aws_latest/aws/sample/gstenc.gst

#filter glimv win -i img -o
#fset win width $width height $height
cyc 0.033

go
