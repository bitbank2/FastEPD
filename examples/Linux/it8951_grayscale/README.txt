Important!!!
In order for this code to work on the Raspberry Pi, the CE0 (chip select0)
line of SPI0 must be changed/disabled in /boot/firmware/config.txt

Add this line:
dtoverlay=spi0-1cs,cs0_pin=27

This will cause it to use BCM 27 (otherwise unused on the IT8951 HAT)

Without this change, the code will not work.

