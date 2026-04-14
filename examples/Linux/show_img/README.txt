show_img
--------
A tool to display JPEG, PNG and BMP images on epaper and framebuffer displays.
Image can be any bit depth and those with 2 or more bits per pixel will be converted to either 2-bit grayscale or b/w/r/y depending on the
chosen display.
Images larger than the panel resolution will return an error.

Building
--------
Before building show_img, clone and install the PNGdec library:
git clone https://github.com/bitbank2/PNGdec
cd PNGdec/linux
build
cd ../..
git clone https://github.com/bitbank2/JPEGDEC
cd JPEGDEC/linux
build

This will build and install the imaging libraries into your system.
Next, build the bb_epaper library if you haven't already:
cd ../bb_epaper/rpi
build

Now you're ready to build show_img using the makefile provided. It copies the
executable into your /usr/bin directory so that it can be executed from
the default system path.

Configuring
-----------
show_img tries to read the configuration information from a file called
epaper.json. There are 5 fields that can be defined:
mode - full, fast, partial
adapter - pimoroni, waveshare_v2
panel_1bit - any supported 1-bit panel name (e.g. EP29_128x296)
panel_2bit - any supported 2-bit (grayscale) or 3/4-color panel name
file - the file to display

All of the above can be overridden from command line parameters. e.g.
./show_img file=~/myimage.png mode=fast

One or both of panel_1bit and panel_2bit must be defined as well as an
adapter, display update mode and file to display.

For black/white panels, if both 1-bit and 2-bit panel types are defined
show_img will choose the correct one based on the image given. 1-bit
mode has the advantage that it can do fast and partial updates while
2-bit grayscale mode cannot use these options. All update modes become
'full' when displaying images with more than 1-bit per pixel.

