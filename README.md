RoboRover - an MK802ii/Gentoo based robot using PIC controller
==============================================================

Program(s) used to control a Wowee 'RoboRover' using an MK802ii ('Android tv stick', 
running Gentoo linux) in combination with a PIC 18f4550 based low level controller 
running modified UBW (universal bitwhacker - see my other repo for mods).

This robot was inspired by Raspberry Tank - For more information on the Raspberry 
Tank project, go to http://bit.ly/raspberrytank

rt_http
-------

rt_http is a robot control program with an HTTP server built in (using mongoose).
It takes commands in a specific format on its port as HTTP GET requests, and
uses them to control the robot using my UBW HBridge mods.
It also reads data IR data from 3 IR sensors on the RoboRover chassis, and currently 
sends three IR patterns, two forwards at different intensities, and one backwards.
Because these are different IR 'commands', the controller can determine which is 
being received by what sensors.  The two level IR sends (inspired by the original
RoboRover electronics, and using these electronics) allows simplistic rangefinding,
and the IR sensors being 'left', 'right', and 'rear' allow for some positional 
informaiton to be read.

You can compile it by running "make" in its directory, assuming you have make
installed.  You also need libpthreads, and kernel modules for I2C if you
intend to use the I2C devices.  It should build mongoose automatically,
if not, run "cd mongoose && make linux".


It was designed for use with the Web UI, though you can probably figure out
how to use it without :)

web-ui
------

The 'web-ui' is probably untoughed from ian's original.
I will put up a repo of the current UI, includeing LiveBots.

From Ian's original description:
web-ui is the frontend for rt_http.  It renders a web page that includes the
tank's video feed, plus buttons with Javascript that sends the appropriate
commands to rt_http, and the readings from the sensors.

The web UI comes in two flavours - a laptop/non-touch flavour where tank
commands are activated by holding the mouse down on an icon, and a phone/
tablet flavour where you just click an icon then click Stop to stop.
(Touch devices tend to do a "right-click" action when you click-and-hold,
so this form of interaction doesn't work very well there.

You can run it with any web server such as lighttpd.  If your document root
isn't /var/www, you'll need to edit the directory that rt_http writes its
sensordata.txt file too.

Old Stuff
---------

henglong_test is a program to exercise the tank's drive motors in sequence.

rt_ssh operates each of the tank's primary functions via text entry, e.g.
"W" followed by "Enter" makes the tank move forward a bit.  Designed to be
operated over SSH.
