video/x-raw,format=I420,width=640,height=480,framerate=30/1 ! omxh264enc bitrate=10000000 ! video/x-h264,streamformat=byte-stream ! h264parse ! avimux ! filesink location=test.avi
