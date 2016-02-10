# aws - Automatic Watch System
## Yohei Matsumoto. 

"aws" is  a concurrent processing midleware designed for Autonomouse Watch System.

aws is actually providing only the filter based concurrent processing model and the execution frame work. Many filters with various functions are defined in the system, the filters can be instantiated, connected and executed flexibly by shell script based command system. You can add new filters by inheriting filter base class, implementing initialization/destruction/processing methods and configuring input/output channels. 

## Building aws
First,  

    git clone https://github.com/yhmtmt/aws.git

Then move to the direcotry "aws",  
 
    git clone https://github.com/yhmtmt/CmdAppBase.git

### for Windows  
Windows version is useful if you need to play the videos or render the 3D graphics. Filter "vfile" plays various video files by using Haali Media Splitter and ffdshow. Filter "dswin" creates a window with Direct3D surface and shows the video frames. You can create Direct3D application simply inheriting the f_ds_window class, the class of "dswin", and overriding input handlers and the proc() function.  

1. Install Visual Studio 2010 or later
2. Install the libraries (NOTE: Basically, the libraries should be built with same version of VC++. vc2010, vc2012, and vc2013 are corresponding to vc10,vc11, and vc12 respectively. Of course, if you are targetting x64, the libraries should be the same.)
   * DirectX SDK (required even if your WindowsSDK may have DirectX libraries)
   * Windows SDK
   * pthread for windows
   * libjpeg (only if you need to use SANYO HD5400)
   * curl (only if you need to use SANYO HD5400)
   * OpenCV 2.4.10
   * PvAPI (NOTE: Need to execute installer. Remove vimba from the system)
   * GLFW, GLEW, freeGLUT
3. Create New empty project for "console application" (I recommend you to configure the compile target as x64 right after creating the project.)
4. Add sources to the project obtained with git clone above explained.(Notice: Files under "rcmd/" should not be imported. The remote commands can only be built with cygwin and Linux.)
5. Configure VC++ "include" and "library" paths. 
6. Install dlls of the libraries to your execution path.
7. Add grobal preprocessor definitions, FWINDOW and AVT_CAM, if you use direct show based window filters ("dswin" and the children) and Allied Vision Technology's cams ("avtcam"). If you use SANYO HD5400(filter "netcam"), you should define SANYO_HD5400. But, in this case, you need to install libjpeg and libcurl correctly.
8. Build the project.
9. Install cygwin with gcc.
10. In the cygwin's shell, move to "rcmd" directory in the source tree, then type 'make" and 'make install "INST_DIR=~/bin". Of course you should create directory "~/bin" preliminary. 
11. Move to the "sample" directory on the cygwin's shell.
12. Execute aws
13. Execute sample.aws on the cygwin's shell.  

Finally you could see the sample filter reports its internal parameter values periodically. If you need to play your video sources following software would be helpful.   

* Haali Media Splitter
* ffdshow  

All video codecs supported by ffdshow can be used through filter "vfile". Note that you need to install x64 ffdshow if you are building aws for x64 environment.


### for Linux  
1. Install the libraries. You need to install them to the following paths relative to the source path to use attached Makefile. Here `$(CPU)` is one of `{arm, x86, x64}`.  

* OpenCV 2.4.10   
 Place headers and libs(so) in the following paths. (relative path)
  * INCLUDE: `opencv/include"
  * LIB: `opencv/$(CPU)/lib`  
* PvAPI  
Place headers and libs(so) in the following paths.
      * INCLUDE: `PvAPI/include`
      * LIB: `PvAPI/bin/$(CPU)`  
* OpenGL, GLFW, GLEW, freeGLUT
Install these OpenGL related libraries, and make sure the paths for include headers and libraries are visible from your linker.
For linux, you can build binary simply typing make.  

    make "CPU=x64"

You can choose cpu type from x64, x86, and arm. For Zynq with OSL, first configure environmental variables of Xilinx's tools, then type,  

    make "ZYNQ=y"
    
Remote commands are also built simultaneously.  The binaries can be installed by  
    make install

to the directory "bin". The directory should be prepared by yourself. Otherwise, you can specify the install directory as follow,  

    make install "INST_DIR=/usr/local/bin"


## Building commands 
aws can only be controled by dedicated remote commands. The remote commands connect to the aws process with TCP socket and play their roles.  You can build them at directory rcmd.  
 
    cd rcmd
    make
 
For Petalinux, after configuring environmental variables of Xilinx's tools,   
 
    make "ZYNQ=y"
 
And the executables built in the directory should be moved to your executable path.  

    make install "INST_DIR=/usr/local/bin"



## Using aws 
1. Build aws and commands.(move the executables to the executable paths)
2. Prepare a home directory, and generate a configuration file named ".aws"
3. Execute commands or the script to build and run the filter graph.

## ".aws" file specification
".aws" file is the configuration file for aws system. Actually, there is a single line description that specifies the destination IP address and the port of the aws commands send their commands. The description is,
    
    rcmd <IP address> <port>
    
By default aws opens port at 20000 on the executing computer.You can command different aws processes by moving to the directory with different ".aws" file. 

You can find the sample of ".aws" at the directory "dbdir". 

## Executing aws
"aws" can be executed with some options.

* `-port <port number>`  
specifies the number of command waiting port. (default 20000)

* `-wpath <path>`  
specifies the path to the working directory. aws uses relative paths from the path to find files specified. This keeps the compatibility between Linux and Windows.(default working path aws executed)

* `-tzone <minute>`  
specifies time zone in minute.  For example, UTC+9 is 540. aws uses UTC time inside the kernel, but the filters often do not. Therefore, aws provides time zone setting to provide local time for each filter.  (default 540)
     
## Building filter graph
You need to build filter graph for your specific application. Filter graphs are composed by  filters and channels. All filters have input and output channels to transfer data from or to other filters. 

There are 2 commands to build filter graph.

* `filter <class name> <instance name> -i <input channel instance#1> ... <input channel instance#n> -o <output channel instance#1> ... <output channel instance#m>`  
This command instantiates filter class specified. The channel instances listed after -i and -o should be instanciated using channel command preliminary.

* `channel <class name> <instance name>`  
This command instantiates channel class specified. Each filter has its own communication channel. The channel instances are to be instantiated before instantiating filters use them.

## Configuring filter parameters
Filters have their own parameters.You need to modify these parameters to control the behaviours of the filters, or you need to get the parameter values to know the processing results of the filter exectuion.  You can set or get the values of the parameters by using commands "fset" and "fget".

* `fset <filter instance name> <parameter name#1> <value#1> <parameter name#2> <value#2> ..... <parameter name#n> <value#n>`  
Setting the filter parameter. You can specify arbitrary number of combinations of the parameter name and the value. (However actually the number is limited by the character length: the length should be less than 1024.)
* `fget <fitler instance name> <parameter name#1> <parameter name#2> ..... <parameter name#n>`  
Getting the filter parameters. You can specifiy multiple parameters, and the parameters are returned as space separated strings.
    
## Running filter graph
To run the filter graph, there are some commands to note.

* `cyc <time in second>`  
specifying the cycle time of the filter execution. (default 1/60 sec) This paramter should be specified befor running filter graphs.

* `trat`  
Time rate specification. Only for offline mode, the time passes specified rate to the actual speed. (default 1) If you want to execute graph faster, please specify the integer value larger than 1.

* `online <yes | no>`  
There two modes running filter graphs; One is online, another is offline. In the offline mode, pause state and step execution is supported. Step execution is useful during designing algorithms which cannot be ran at realtime. "pause" and "step" are described later. The mode should be specified before running filter graph.
    
* `go [<start time> [<end time>]]`  
Desc: Running filter graph. For online mode, the command simply execute filter graph. For ofline mode, The filter graph is executed from the specified start time and to the end time. When the time is reached to the end time specified, the filter graph transits the state to "pause". Time should be specified aws's common format.(See Etc)
    
* `pause`  
If the online mode is enabled, you can pause the execution by this command. The state is again back to running state by executing "go" command without specifying start and end time. "step" command can be used to run graphs few cycles and pause again.
    
* `step [<absolute time> | c <number of cycles>]`  
For pause state, you can step to the specified time. If no argument is specified, step run one cycle from the current time. If a argument <absolute time> is specified, the filter graph jumps to the specified time. Finally if <number of cycles> following after "c", the filter graph jumps to specified cycles later.
    
* `stop`  
Stopping filter graphs. 

* `quit`  
Shutdown aws process. 

* `finf [{n <filter id> | <filter name>}]`  
Getting fitler information. If no argument specified, the nubmer of filters registerd in the process is returned. If `n <filter id>` is specified, the filter information is returned according to the id. If `<filter name>` is specified the filter information is returned according to the name. The filter information is currently set as `<name> <id> <num params> <num input chans> <num output chans>`. Using this information, you can enumerate further information related to the filter. Of course, I'm planning to implement new commands as they are.  
* `fpar <filter name> <parameter id>`
Getting parameter information of the filter specified with the name. `<parameter id>` should be less than the number of parameters in the filter. The number of parameters in the filter can ge got by finf.  
* `chinf [{n <channel id> | <channel name>}]`  
Getting channel information. If no argument specified, the number of channels defined in the process is returned. Otherwise, the channel information is returned according to the specified `<channel id>` or `<channel name>`. The information of the channel is currently set as `<channel name> <channel id>`.   
* `fls`
Getting the list of filters instantiated in the process.
* `chls`
Getting the list of channels instantiated in the process.
* `fpls <filter name>`
Getting the list of parameters in the filter instance named `<filter name>`.
* `frm <filter name>`  
Removing the filter specified.
* `chrm <channel name>`  
Removing the filter specified.
* `awsevt [-f <event filter>] [-t <time>] [-to <time out>] [-sock <host> <port>]`  
Register and wait a timer event. The command need a event filter `evt` to be instantiated as `<event filter>`. `<time>` is the absolute time format described below.Even if the event is not arrived, the command returns after `<time out>` passed. Here `<time out>` should be specified in second and is 5 seconds in default. `<host>` and `<port>` are the destination UDP socket of the event notification.
## Filters
Here I describe the filter classes currently included in the system. 

1. sample  
Sample of the filter design. You will understand how the new filter can be implemented. This class is defined in f_base.h as f_sample.  
	* IN: Null  
	* OUT: Null  
	* PAR:  
f64par : 64bit floating point number  
s64par : 64bit signed integer  
u64par : 64bit unsigned integer  
	* PRC: Only printing values of parameters to stdout.  

2. nmea  
IO source filter of NMEA0183. The input source can be serial ports, UDP sockets and files. File input is only supported for offline mode, and the file format should be "<time> <NMEA sentence>". <time> should be specified aws's common format.(See Etc.)  
	* IN:  nmea
	* OUT: nmea 
	* PAR:  
fnmea: File path of NMEA source file  
src_host: IP address of NMEA source (if not specified ADDR_ANY is used)  
dst_host: IP address of NMEA destination (if not specified UDP output is not turned on).  
src: Source type. One of {FILE,  UDP, COM}.  
com: Number of NMEA source COM port.  
bps: Baud rate of NMEA source COM port.  
port: Port number of NMEA source UDP.  
log: Log enable (y or n)  
filter: Sentence filter. 5 characters are to be specified. \* can be used as wild card.  
	* PRC:  
Write nmea in the input channel to the IO source. 
Read nmea in the IO source and write it to output channel.
3. nmea_proc  
This filter decode NMEA0183 and set the sensor variables to the system's global variables. Input is from filter "nmea" with channel "nmea". Currently, a part of GPS and AIS sentences are supported.   
   * IN: nmea  
   * OUT: NULL  
   * PAR:
bm_ver: The version of the AIS binary message which aws defined.  
trmc: {yes, no} Correct the time of the system with RMC sentence.  
   * PRC:

3. stab  
This filter stabilizes the rigid motion (means 2D rotation and translation). Rotation and vertical motions are removed. Of course, we cannot distinguish true camera motion to unintended motion. We need to choose carefully the sensitivity constant of the algorithm. The filter requires a color image and the grayscale image as the input channels. Then the filter outputs stabilized color and grayscale images. (This is not designed for new framework, and should be modified.)
   * IN: img(grayscale) img(color)
   * OUT: img(grayscale) img(color)
   * PAR:  
roi: ROI for template. (4 integer values)  
plv: Pyramit level.  
itr: Maximum number of Gauss-Newton iteration.  
log: Loggin.  
clr: Initialize grayscale output to grayscale input. (This cause the template region without stabilization. Thus it initializes the algorithm)  
w: flag for weighted iteration  
m: load mask image to eliminate disturbing pixels from calculation.  
rb: 
rbth:  
rsbsz:  
wt: Warp type.  
ipt: inter polation type
alhpa:  
beta:  
through: Disable stabilization.
disp: Displaying stabilization status.  
   * PRC:

4. shioji_ctrl_rcv  
The filter receives Shiojimaru's control packet from the UDP socket. (This filter is out of date. It needs to be modified for current framework)  
   * IN:
   * OUT: 
   * PAR:
   * PRC:
 
5. shioji_ctrl  
This filter output control values to the filter "shioji". The values are modified via remote command.(This filter is out of date. It needs to be modified for current framework.)
   * IN:
   * OUT: 
   * PAR:
   * PRC:
 
6. shioji  
This filter sends packets to control Shiojimaru. The control values are given both from the input channel and remote command. This filter also recieves Shiojimaru's state packet transmitted from the server, and output the values to the channel. (This filter is out of date. It needs to be modified for current framework.)
   * IN:
   * OUT: 
   * PAR:
   * PRC:
 
7. clip
   * IN:
   * OUT: 
   * PAR:
   * PRC:

8. avt_cam (beta)  
This filter grabs images from AVT cameras. GT2750C and GT1920C are tested (Very thanks to Takeshi Ohkawa).  
   * IN:
   * OUT: imgr 
   * PAR:  
host: IP addres of the AVT camera  
StreamBytesPerSecond: Bandwidth limitation. (default 15000000)  
update: y updates dynamic parameters y or n
   * PRC:
Grabber threads are invoked independent of proc().  proc() only updates dynamic parameters when update is commanded.
9. imgs
   * IN:
   * OUT: 
   * PAR:
   * PRC:
 
10. dwin
   * IN:
   * OUT: 
   * PAR:
   * PRC:

11. syswin
   * IN:
   * OUT: 
   * PAR:
   * PRC:

12. spwin
   * IN:
   * OUT: 
   * PAR:
   * PRC:

13. ptzwin
   * IN:
   * OUT: 
   * PAR:
   * PRC:
 
14. inspector
   * IN:
   * OUT: 
   * PAR:
   * PRC:

15. vfile
This filter grabs images from video file via DirectShow filters. Only for Windows.  
   * IN:
   * OUT: {imgr or imgc}  
   * PAR:  
file: File path of the video.  
abs_time: Absolute time of the video start. Follow the time specification format described later.  
   * PRC:
 
16. vdev
This filter grabs images from video capture device via DirectShow filters. Only for Windows  
   * IN:
   * OUT: {imgr or imgc}  
   * PAR:
device: device number. (default 0)  
   * PRC:
 
17. uvcam
This filter grabs images from USB camera compatible with UVC driver. Only for Linux. Logicool C270 is tested.   
   * IN:
   * OUT: {imgr or imgc}  
   * PAR:  
dev_name: Path to the camera device. (default /dev/video0)  
   * PRC:
Grabbing a image from camera and transmitting it to output channel.  

18. trnimg  
Transmits images over TCP/IP network. You can choose the compression algorithm, color format, color depth, image scale, and compression quality.
	* IN: {imgc | imgr}  
	* OUT:   
	* PAR:
port: Destination port number  
fmt: Image data format {0: raw 1: jpg 2: png)  
depth: Color depth in byte  
channel: Number of color channels  
fmt: Color format {0: Mono, 1: Bayer, 2:RGB}  
qjpg: Jpeg quality [0-100]")  
qpng: PNG quality [0-10]"  
scale: Scale for resizing.  
	* PRC:
First, Waiting for connection to rcvimg instance.
After the session established, images in the input channel is sent to the rcvimg instance with specified image format and scale.  
19. rcvimg  
 Recieves images transmitted by trnimg instances. Basically image format is recognized automatically by source packet's format fields.
	* IN:  
	* OUT: {imgc | imgr}  
	* PAR:  
addr: Server address (in IPv4)  
port: Destination port number  
fmt: Image data format {0: raw 1: jpg 2: png)  
depth: Color depth in byte  
channel: Number of color channels  
cfmt: Color format {0: Mono, 1: Bayer, 2:RGB}  
qjpg: Jpeg quality [0-100]  
qpng: PNG quality [0-10]  
      * PRC:  
First Connecting to trnimg instance. 
After the connection established, images are received and transfered to output channel.  
20. debayer 
    * IN: {imgc | imgr}
    * OUT: {imgc | imgr}
    * PAR:  
type: Bayer type for both 8bit and 16bit. {BG8, GB8, RG8, GR8, BG16, GB16, RG16, GR16}  
      * PROC:  
The image in the input channel is converted to BGR, and stored to the output channel. Bit depth is not changed.

21. imwrite 
    * IN: {imgc | imgr}
    * OUT: 
    * PAR:  
type: Image format {tiff, jpg, png}  
qjpg: Jpeg quality. [0-100]  
qpng: Png quality. [0-10]  
path: Storing path.  
      * PRC:  
Images in input channel is saved as files with specified image format. File name is the combination of the filter name and time record.

22. fep01
    * IN: {crbuf | crbuf2k | crbuf4k | crbuf8k}
    * OUT: {crbuf | crbuf2k | crbuf4k | crbuf8k}
    * PAR:  
port: Number of serial port (for windows)  
dev: Device file of serial port. (for Linux)  
br: Baudrate of the serial port.  
lpkt: Packet length the FEP01 uses.[0-128] (Maybe packet length larger than about 100 could be collapse the buffer.9  
cm: Communication mode. {p2p} (Currently only p2p is supported.)  
thead: Time stamp is added at header. {yes | no}  
addr_p2p: Destination address in p2p mode.  
ftxlog: Path for transmission log file.  
frxlog: Path for recieving log file.  
st: State configuration. {init, rst, op, dbg, test, tcl, tsv}  
addr: Own addres. [0-240]  
rep: Acknowledge is confirmed for each data transmission. {yes | no}  
rep_power: Get power report when the packet is recieved. {yes | no}  

    * PRC:  
Some sample scripts are in sample/fep01/. In state "init", ARG command is issued and the returned values are stored into filter parameters. In state "rst", filter parameters are written to the device's register and then RST command is issude. In state "op", The data in the input channel is transmitted automatically to the destination specified, and the recieved data is automatically copied to the output channel. In "dbg" mode, FEP01's commands can be manually issued using some filter parameters. In "tcl" state, it works as time synchronization client waiting for time information packet, from time synchronization server FEP01 configured as state "tsvr".

23. ser
    * IN: {crbuf | crbuf2k | crbuf4k | crbuf8k}
    * OUT: {crbuf | crbuf2k | crbuf4k | crbuf8k}
    * PAR:  
*port: Number of serial port (for Windows)  
*dev: Device file of the serial port (for Linux)  
*br: Baud rate of serial port  
    * PRC:  
The data in the input channel is transmitted to serial port specifiedd. And the recieved data is copied to the output channel.
24. udp
    * IN: {crbuf | crbuf2k | crbuf4k | crbuf8k}
    * OUT: {crbuf | crbuf2k | crbuf4k | crbuf8k}
    * PAR:  
* port_dst: Destination port number
* host_dst: Host address of the destination.
* lpkt: Length of the packet.
* fout: Output log file.
* fint: Input log file.

    * PRC:  
The data in input channel is transmitted to UDP destination specified. And the recieved data is copied to the output channel.
25. dd
    * IN: Null
    * OUT: {crbuf | crbuf2k | crbuf4k | crbuf8k}
    * PAR:
    * PRC:  
Generate dummy data and output it to the output channel.
26. rd
    * IN: {crbuf | crbuf2k | crbuf4k | crbuf8k}:
    * OUT: Null
    * PAR:
    * PRC:  
Record the data in the input channel.

# Channels
1. imgc  
 Transfers Mat object. Destination filter gets the clone of the image object.
	* Pars Mat  
 
2. imgr  
Transfers Mat object. Destination filter gets the reference of the image object. You need to be careful if the channel is connected to multiple destinations.  
	* Pars Mat  
 
3. nmea  
Transfers nmea sentences. Source filter pushes nmea sentences to the channel. Destination filters pops them.
	* Pars vector<char[83]>  
4. ship_ctrl  
	* Pars
5. crbuf
1KByte Ring buffer. crbuf2k, crbuf4k, crbuf8k are the variant the sizes are 2Kbyte, 4Kbyte, 8Kbyte respectively.

## Designing New Filter
Here I explain how you can design and add your new filter to the system. There are some points. 1. and 2. are the duty, and the others are the tips.

#### Inherit f_base class and implement followings (here the filter class is f_filter)
* `f_filter(const char *): f_base(name){ }`  
The constructor has single "const char *" argument, and the f_base(const char*) should be called in the initialization list as follow,
     
* `virtual bool init_run()`  
Override if you need to initialize the filter before running it.If you return false, the filter graph cannot go to running state.  

* `virtual void destroy_run()`  
Ovverride if you need to destroy something before stopping it.  

* `virtual bool proc()`  
The main function of the filter. The function is iteratively called by the framework during running state. If you return false, the filter graph is stopped totally.

#### Register your filter to the system.
To instantiate your filter with "filter" command, you need to register it to the system by inserting a registration code. In the file "factory.cpp", first, you need to include a header file includes your filter class. Then, please insert the following code to f_base::register_factory().
 
    register_factory<f_filter>("filter")
     
Where "filter" is the name of the filter class. 

#### Declare parameters and register them if you need to get or set their values from outside the process.
You can expose most of the types of parameters to outside of the system easily by calling "register_fpar()" series inside the constructor "f_filter(const char*)". For most of the  parameter types, "register_fpar()" is called as follow,

    register_fpar("parameter_name", &parameter, "Here is the parameter explanation.");
 
Then you can access the parameter using fset/fget command with the parameter name "parameter_name". 

For string parameters, you need to allocate sufficient memory area and then,  
 
    char_str = new char[1024]; // here char_str is "char *" declared in the class.
    register_fpar("char_str", char_str, 
        1023 /* buffer length without null character*/, 
        "This is the string parameter sample.");
 
You can specify the enum parameter corresponding to string set. First, prepare the enum and string set.  
 
    enum e_val {
         ALPHA, BETA, GAMMA, UNKNOWN 
    };

    char * estr[UNKNOWN] = {
         "alpha", "beta", "gamma"
    };
 
Then at "f_filter(const char *)",  

    register_fpar("eval", (int*)(eval) /* e_val parameter */, 
        estr, "Choose from {alpha, beta, gamma}");
 
where eval is the e_val type parameter declared in the class.

#### get channel connecting to other filters.
You may need to access data channels connecting to other filters inside "proc()"". Input and output channels are listed in m_chin and m_chout respectively, and their type is vector<ch_base*>. You should use dynamic_cast to cast the channel objects to yours. Framework of aws does not have the validation schemeof the channel list, so you need to check the list at "init_run()" or "proc()" to be what you intended.

#### get current time.
In some cases, you need to watch the current time, you can get it from "m_cur_time" member. And also you can get time string for specified time zone by "get_time_str()". get_time_str() is useful for logging with time record.

#### Make your filter independent
Basically, your filter should work independently if the other filters caused the problem.  Because the dependencies between filters are limited to the channels, only what you need to do is correctly handling the channels.
* When required channel instances are not registered in m_chin and m_chout, the filter should return false at "init_run()"
* Even if channel does not have valid data, "proc()" must not be stopped. You should clearly define the behavior of "proc()" for the case. The channel data transmission can be delayed in the filter graph.
* Prepare for the pause state. If your filter is used in the offline mode, 

## Designing New Channel
You may need to design new channels to connect your own filters. 

#### Inherit ch_base and implement followings. (here your channel class is ch_channel)
* `ch_channel(const char * name): ch_base(name){ }`  
The constructor has a "const char * " argument, and the initialization list has ch_base initialization.  

#### Insert instantiation code to the factory function
Include your header file defines your own channel in "factory.cpp" and insert following code to "ch_base::register_factory()"  
     
    register_factory<ch_channel>("channel");
     
#### Define and implement your setter/getter with mutual exclusion.
You can add your own setter/getter function. Be careful that the channel can be accessed from multiple filters simultaneously. The setter/getter should correctly use the mutual exclusion methods "lock()" and "unlock()" prepared in ch_base().


## Utilities

## Etc

* In the framework, time specification is in the form of `[<week day> <month> <day> <hour>:<minute>:<second>:<millisecond> <year>]`. `<week day>` is in { Sun, Mon, Tue, Wed, Thr, Fri, Sat }, <month> is in { Jan, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Nov, Dec}, `<day>`, `<hour>`, `<minute>` and `<second>` are two digits (means zero should be padded for the single digit day.),  `<millisecond>` is three digits, and <year> is four digits. Here is the example,  
    
    [Sun Aug 17 21:07:38.072 2014]
     
This format is actually the same as TeraTerm time stamp. So you can use TeraTerm for loggin Serial communications such as NMEA0183. 

## License
Source codes are under GPLv3.  

[GPLv3](http://www.gnu.org/licenses/gpl-3.0.txt)