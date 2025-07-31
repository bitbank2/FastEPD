![FastEPD raw epaper component](https://github.com/user-attachments/assets/9c59164f-326c-46bf-b210-54b0e4d35c57)

Download and render image example
=================================

In my initial days of collaboration with the epdiy library circa 2021 we discussed  the idea of adding an WiFi download and JPEG render example.

  **[jpg-render.c](https://github.com/vroland/epdiy/blob/main/examples/www-image/main/jpg-render.c)**
  My original example used the ESP32 tjpgd that is in the ROM. It takes between 20 and 30% more time to decode than the jpegdec component. This is not in this repository since we took the fastest possible decoding.
  
  **jpgdec-render.cpp**
  This version uses [Bitbank2 jpeg decoder](https://github.com/bitbank2/JPEGDEC) as an external component, please run: **git submodule update --init --recursive**
  in order to download it and it will be placed in components/jpegdec. Using Larry component decoding is faster and in this particular example we are also not wasting any time drawing per pixel, we just copy directly the decoded image, in the 4 BPP epaper framebuffer in PSRAM.
  
  Copying the entire rows to the framebuffer reduces almost completely the rendering time at the cost of loosing software rotation and gamma correction, but that might be not needed if you want to render an image as fast as possible.

Detailed statistics using jpgdec-render.cpp:

```
164808 bytes read from http://img.cale.es/jpg/fasani/5e5ff140694ee
decode: 453 ms - 1200x825 image MCUs:52 
www-dw: 1219 ms - download 1024 buffer
render: 0 ms - copying pix (memcpy)
```

**Note:** Statistics where taken with the 9.7" display which has a resolution of 1200*825 pixels and may be significantly higher using bigger displays.

Building it
===========

Do not forget to update your WiFi credentials and point it to a proper URL that contains the image with the right format:

```c
// Edit the file: main/settings.h
// WiFi configuration
#define ESP_WIFI_SSID     "WIFI NAME"
#define ESP_WIFI_PASSWORD ""
// Add your display dimensions:
#define EPD_WIDTH 1200
#define EPD_HEIGHT 825
// This is then also used here:
#define IMG_URL "https://loremflickr.com/" STR(EPD_WIDTH) "/" STR(EPD_HEIGHT)
// You can of course point that IMG_URL to your own image source or service
```

Note that as default an random image taken from loremflickr.com is used. You can use any URL that points to a valid Image, take care to use the right JPG format, or you can also use the image-service [cale.es](https://cale.es) to create your own gallery.

**Important note about secure https**
Https is proved to work on stable ESP-IDF v4.2 branch. Please Note that for IDF versions >= 4.3 it needs to have VALIDATE_SSL_CERTIFICATE set to true. 
In case you want to allow insecure requests please follow this:

 In menu choose Component config->ESP LTS-> (enable these options) "Allow potentially insecure options" and then "Skip server verification by default"

Also needs the main Stack to be bigger otherwise the embedTLS validation fails:
Just 1Kb makes it work: 
CONFIG_ESP_MAIN_TASK_STACK_SIZE=4584

You can set this in **idf.py menuconfig**

     -> Component config -> ESP32-specific -> Main task stack size

Setting VALIDATE_SSL_CERTIFICATE to false also works skipping the .cert_pem in the esp_http_client_config_t struct. 


Happy to collaborate with this amazing project,

Martin Fasani, 
Barcelona. January of 2025
