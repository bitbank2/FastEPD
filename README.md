BB_EPDIY
========
A complete solution for driving parallel eink displays (think Kindle)<br>
Written by Larry Bank<br>

<b>Why did you write it?</b><br>
I'm fascinated by displays of all types, especially eink. I already wrote a library for SPI type displays (built-in controllers) called bb_epaper. Recent client projects involving parallel eink displays have inspired me to take a look at what it takes to drive them. At the time of this writing, most ESP32 projects which use parallel eink displays will make use of the EPDiy library. This library has been in existence for about 5 years and has grown to support a wide range of devices. For my use cases, the library is missing features and is complicated to modify, so I decided to re-invent the wheel.<br>
<b>What's special about bb_epdiy?</b><br>
Since writing several display libraries (OneBitDisplay, bb_spi_lcd, bb_epaper), I've used all of my experience with graphics and C++ APIs to create a clean design from the start. Libraries often grow in random ways over time as new features are bolted on and design problems are covered up with hacks. In this case, bb_epdiy has a better design from the begining and will be easier to maintain and understand. Some of the unique features are compressed fonts and graphics, more flexibility in display updates and a simple API which includes pre-configured setups for popular hardware.

Please read the Wiki for API details.

![bb_epdiy](/bb_epdiy_gif.jpg?raw=true "bb_epdiy")

If you find this code useful, please consider becoming a sponsor or sending a donation.

[![paypal](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=SR4F44J2UR8S4)

~                                                                                      
