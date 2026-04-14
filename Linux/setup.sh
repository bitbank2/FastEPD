# Install and build the required libraries
cd ~
mkdir Projects
cd Projects
git clone https://github.com/bitbank2/JPEGDEC
cd JPEGDEC/linux
make
cd ../..
git clone https://github.com/bitbank2/AnimatedGIF
cd AnimatedGIF/linux
make
cd ../..
sudo apt install gpiod libgpiod-dev libdrm-dev

