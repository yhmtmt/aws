# aws - Automatic Watch System
## Yohei Matsumoto. 

"aws" is  a concurrent processing midle designed for Autonomouse Watch System.

aws is actually providing only the filter based concurrent processing model and the execution frame work. Many filters with various functions are defined in the system, the filters can be instantiated, connected and executed flexibly by shell script based command system. You can add new filters by inheriting filter base class, implementing initialization/destruction/processing methods and configuring input/output channels. 

## Building aws
aws depends on various libraries. 

* DirectX SDK (For Windows)
* Windows SDK (For Windows)
* curl (For Windows)
* pthread for windows (For Windows)
* libjpeg (for Windows)
* OpenCV 2.4.9
* cminpack 
* PvAPI  
  
For linux, I prepared a Makefile. You can build binary simply typing make.  

    make

For Petalinux@Zynq, first configure environmental variables of Xilinx's tools, then type,  

    make "ZYNQ=y"
    
## Building commands 
aws can only be controled by dedicated remote command. You need to build them at directory rcmd.  
 
    cd rcmd
    make
 
For Petalinux, after configuring environmental variables of Xilinx's tools,   
 
    make "ZYNQ=y"
 
And the executables built in the directory should be moved to your executable path.

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

*`-wpath <path>`  
specifies the path to the working directory. aws uses relative paths from the path to find files specified. This keeps the compatibility between Linux and Windows.(default working path aws executed)

*`-tzone <minute>`
specifies time zone in minute.  For example, UTC+9 is 540. aws uses UTC time inside the kernel, but the filters often do not. Therefore, aws provides time zone setting to provide local time for each filter.  (default 540)
     
## Building filter graph
You need to build filter graph for your specific application. Filter graphs are composed by  filters and channels. All filters have input and output channels to transfer data from or to other filters. 

There are 2 commands to build filter graph.

(1) "filter" command
    filter <class name> <instance name> -i <input channel instance#1> ... <input channel instance#n> -o <output channel instance#1> ... <output channel instance#m>`
Desc: This command instantiates filter class specified. The channel instances listed after -i and -o should be instanciated using channel command preliminary.
    
    

(2) "channel" command
    channel <class name> <instance name> 
Desc: This command instantiates channel class specified. Each filter has its own communication channel. The channel instances are to be instantiated before instantiating filters use them.

## Configuring filter parameters
Filters have their own parameters.You need to modify these parameters to control the behaviours of the filters, or you need to get the parameter values to know the processing results of the filter exectuion.  You can set or get the values of the parameters by using commands "fset" and "fget".

(1) "fset" command
Desc: Setting the filter parameter. You can specify arbitrary number of combinations of the parameter name and the value. (However actually the number is limited by the character length: the length should be less than 1024.)
    
Usage:  
    fset <filter instance name> <parameter name#1> <value#1> <parameter name#2> <value#2> ..... <parameter name#n> <value#n>
    
(2) "fget" command
Desc: Getting the filter parameters. You can specifiy multiple parameters, and the parameters are returned as space separated strings.
    
Usage:  
    fget <fitler instance name> <parameter name#1> <parameter name#2> ..... <parameter name#n>
    
## Running filter graph
To run the filter graph, there are some commands to note.

(1) cyc
Desc: specifying the cycle time of the filter execution. (default 1/60 sec) This paramter should be specified befor running filter graphs.
    
Usage:  
    cyc <time in second>
    
(2) syn
Desc: This parameter is currently not working.
    
Usage:  
    syn
    
(3) trat
Desc: Time rate specification. Only for offline mode, the time passes specified rate to the actual speed. (default 1) If you want to execute graph faster, please specify the integer value larger than 1.
    
Usage:  
    trat <time rate>
    

(4) online
Desc: There two modes running filter graphs; One is online, another is offline. In the offline mode, pause state and step execution is supported. Step execution is useful during designing algorithms which cannot be ran at realtime. "pause" and "step" are described later. The mode should be specified before running filter graph.
    
Usage:  
    online <yes | no>
    
(5) go 
Desc: Running filter graph. For online mode, the command simply execute filter graph. For ofline mode, The filter graph is executed from the specified start time and to the end time. When the time is reached to the end time specified, the filter graph transits the state to "pause". Time should be specified aws's common format.(See Etc)
    
Usage:  
    go [<start time> [<end time>]]
    
(6) pause
Desc: If the online mode is enabled, you can pause the execution by this command. The state is again back to running state by executing "go" command without specifying start and end time. "step" command can be used to run graphs few cycles and pause again.
    
Usage:  
    pause
    
(7) step
Desc: For pause state, you can step to the specified time. If no argument is specified, step run one cycle from the current time. If a argument <absolute time> is specified, the filter graph jumps to the specified time. Finally if <number of cycles> following after "c", the filter graph jumps to specified cycles later.
    
Usage:  
    step
    step <absolute time> 
    step c <number of cycles>
    
(8) stop
Desc: Stopping filter graphs. 
    
Usage:  
    stop
    
(9) quit
Desc: Shutdown aws process. 
    
Usage:  
quit
    
## Fitlers
Here I describe the filter classes currently included in the system. 

(1) sample

Desc: Sample of the filter design. You will understand how the new filter can be implemented. This class is defined in f_base.h as f_sample.
 
-IN: Null
 
-OUT: Null
 
-PAR: 
f64par : 64bit floating point number 
s64par : 64bit signed integer 
u64par : 64bit unsigned integer
 
-PRC: Only printing values of parameters to stdout.

(2) nmea
 
Desc: IO source filter of NMEA0183. The input source can be serial ports, UDP sockets and files. File input is only supported for offline mode, and the file format should be "<time> <NMEA sentence>". <time> should be specified aws's common format.(See Etc.) 
 
-IN:  nmea
 
-OUT: nmea 
 
-PAR:
fnmea: File path of NMEA source file
src_host: IP address of NMEA source (if not specified ADDR_ANY is used)
dst_host: IP address of NMEA destination (if not specified UDP output is not turned on). 
src: Source type. FILE or UDP or COM is allowed.
com: Number of NMEA source COM port.
bps: Baud rate of NMEA source COM port.
port: Port number of NMEA source UDP.
log: Log enable (y or n)
filter: Sentence filter. 5 characters are to be specified. * can be used as wild card.
 
-PRC:
Write nmea in the input channel to the IO source. 
Read nmea in the IO source and write it to output channel.

 
(4) hd5400
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:

 
(5) imgshk
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(6) debayer
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(7) gry
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(8) edge
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(9) reg
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(10) hough
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(11) bgksub
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(11) stab
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(12) window
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(13) mwin
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(14) shioji_ctrl_rcv
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(15) shioji_ctrl
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(16) shioji
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(17) shipdet
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(18) trck
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(19) gauss
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(20) camcalib
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(21) clip
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(22) avt_cam
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(23) imgs
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(24) dwin
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(25) syswin
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(26) spwin
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(27) ptzwin
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(28) inspector
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(29) vfile
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(30) vdev
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(31) uvcam
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(32) trnimg
 
-Desc: Transmits images over TCP/IP network. You can choose the compression algorithm, color format, color depth, image scale, and compression quality.
 
-IN: {imgc | imgr}
 
-OUT: 
 
-PAR:
port: Destination port number
fmt: Image data format {0: raw 1: jpg 2: png)
depth: Color depth in byte
channel: Number of color channels
fmt: Color format {0: Mono, 1: Bayer, 2:RGB}
qjpg: Jpeg quality [0-100]")
qpng: PNG quality [0-10]"
scale: Scale for resizing.
 
-PRC:
First, Waiting for connection to rcvimg instance.
After the session established, images in the input channel is sent to the rcvimg instance with specified image format and scale. 

 
(32) rcvimg
 
-Desc: Recieves images transmitted by trnimg instances. Basically image format is recognized automatically by source packet's format fields.
 
-IN:
 
-OUT: {imgc | imgr}
 
-PAR:
addr: Server address (in IPv4)
port: Destination port number
fmt: Image data format {0: raw 1: jpg 2: png)
depth: Color depth in byte
channel: Number of color channels
cfmt: Color format {0: Mono, 1: Bayer, 2:RGB}
qjpg: Jpeg quality [0-100]")
qpng: PNG quality [0-10]"
 
-PRC:
First Connecting to trnimg instance. 
After the connection established, images are received and transfered to output channel.

 
(32) trn
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:
 
(32) rcv
 
-Desc: 
 
-IN:
 
-OUT: 
 
-PAR:
 
-PRC:


# Channels
 
(1) imgc
 
-Desc Transfers Mat object. Destination filter gets the clone of the image object.
 
-Pars Mat
 
(2) imgr
 
-Desc Transfers Mat object. Destination filter gets the reference of the image object. You need to be careful if the channel is connected to multiple destinations.
 
-Pars Mat
 
(1) pvt
 
-Desc
 
-Pars
 
(1) nmea 
 
-Desc Transfers nmea sentences. Source filter pushes nmea sentences to the channel. Destination filters pops them.
 
-Pars vector<char[83]>
 
(1) ais
 
-Desc
 
-Pars
 
(1) bmsg
 
-Desc
 
-Pars
 
(1) ship
 
-Desc
 
-Pars
 
(1) ship_ctrl
 
-Desc
 
-Pars
 
(1) vrect
 
-Desc
 
-Pars
 
(1) trck
 
-Desc
 
-Pars
 
(1) ptz
 
-Desc
 
-Pars
 
(1) ptzc
 
-Desc
 
-Pars

(1) campar
 
-Desc

-Pars



## Designing New Filter
Here I explain how you can design and add your new filter to the system. There are some points. 1. and 2. are the duty, and the others are the tips.

1. Inherit f_base class and implement followings (here the filter class is f_filter)
* 'f_filter(const char *)'
The constructor has single "const char *" argument, and the f_base(const char*) should be called in the initialization list as follow,
   
    f_filter::f_filter(const char * name): f_base(name)
    {
    }
     

* `virtual bool init_run()`
Override if you need to initialize the filter before running it.If you return false, the filter graph cannot go to running state.

* `virtual void destroy_run()`
Ovverride if you need to destroy something before stopping it.

* `virtual bool proc()`
The main function of the filter. The function is iteratively called by the framework during running state. If you return false, the filter graph is stopped totally.

2. Insert your filter to the factory function.
To instantiate your filter with "filter" command, you need to insert the instantiation code to the factory function. Factory function "f_base * f_base::create(const char * tname, const char * fname)" is in "filter/f_base.cpp". You should add following code,
 
    if(strcmp("filter", tname) == 0){
      return new f_filter(tname);
    }
     
Where "filter" is the name of the filter class. Of course, "f_filter" should be defined in this scope, you need to include your header file at "filter/f_base.h". Also, your filter source code should be placed in the directory "filter".


3. Declare parameters and register them if you need to get or set their values from outside the process.
You can expose most of the types of parameters to outside of the system easily by calling "register_fpar()" series inside the constructor "f_filter(const char*)". For most of the  parameter types, "register_fpar()" is called as follow,

    register_fpar("parameter_name", &parameter, "Here is the parameter explanation.");
 
Then you can access the parameter using fset/fget command with the parameter name "parameter_name". 

For string parameters, you need to allocate sufficient memory area and then,

 
    char_str = new char[1024]; // here char_str is "char *" declared in the class.
    register_fpar("char_str", char_str, 1023 /* buffer length without null character*/, "This is the string parameter sample.");
 

You can specify the enum parameter corresponding to string set. First, prepare the enum and string set.

 
    enum e_val {
         ALPHA, BETA, GAMMA, UNKNOWN 
    };

    char * estr[e_val::UNKNOWN] = {
         "alpha", "beta", "gamma"
    };
 

Then at the "f_filter(const char *)", 

 
    register_fpar("eval", (int*)(eval) /* e_val parameter */, estr, "Choose from {alpha, beta, gamma}");
 

where eval is the e_val type parameter declared in the class.

4. get channel connecting to other filters.
You may need to access data channels connecting to other filters inside "proc()"". Input and output channels are listed in m_chin and m_chout respectively, and their type is vector<ch_base*>. You should use dynamic_cast to cast the channel objects to yours. Framework of aws does not have the validation schemeof the channel list, so you need to check the list at "init_run()" or "proc()" to be what you intended.

5. get current time.
In some cases, you need to watch the current time, you can get it from "m_cur_time" member. And also you can get time string for specified time zone by "get_time_str()". get_time_str() is useful for logging with time record.

6. Filter should be independent.
Basically, your filter should work independently if the other filters caused the problem.  Because the dependencies between filters are limited to the channels, only what you need to do is correctly handling the channels.
* When required channel instances are not registered in m_chin and m_chout, the filter should return false at "init_run()"
* Even if channel does not have valid data, "proc()" must not be stopped. You should clearly define the behavior of "proc()" for the case. The channel data transmission can be delayed in the filter graph.
* Prepare for the pause state. If your filter is used in the offline mode, 



## Designing New Channel
You may need to design new channels to connect your own filters. 

1. Inherit ch_base and implement followings. (here your channel class is ch_channel)
* `ch_channel(const char * name)`
The constructor has a "const char * " argument, and the initialization list has ch_base initialization.

     
    ch_channel(const char * name): ch_base(name)
    {
    }
     

2. Insert instantiation code to the factory function
Include your definition to "ch_base.h" and insert following code to "ch_base::create(const char * type_name, const char * chan_name)"
     
    if(strcmp("channel", type_name) == 0){
      return new ch_channel(chan_name);
    }
     
3. Define and implement your setter/getter with mutual exclusion.
You can add your own setter/getter function. Be careful that the channel can be accessed from multiple filters simultaneously. The setter/getter should correctly use the mutual exclusion methods "lock()" and "unlock()" prepared in ch_base().


## Utilities

## Etc

* In the framework, time specification is in the form of `[<week day> <month> <day> <hour>:<minute>:<second>:<milisecond> <year>]`. `<week day>` is in { Sun, Mon, Tue, Wed, Thr, Fri, Sat }, <month> is in { Jan, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Nov, Dec}, `<day>`, `<hour>`, `<minute>` and `<second>` are two digits (means zero should be padded for the single digit day.),  `<milisecond>` is three digits, and <year> is four digits. Here is the example,

    
    [Sun Aug 17 21:07:38:072 2014]
     

This format is actually the same as TeraTerm time stamp. So you can use TeraTerm for loggin Serial communications such as NMEA0183. 

## License
