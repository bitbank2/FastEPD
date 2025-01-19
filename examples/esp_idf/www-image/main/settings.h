// WiFi configuration:
#define ESP_WIFI_SSID "your_ssid"
#define ESP_WIFI_PASSWORD "your_pw"

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
// Affects the gamma to calculate gray (lower is darker/higher contrast)
// Nice test values: 0.9 1.2 1.4 higher and is too bright
double gamma_value = 0.9;

// Deepsleep configuration
#define MILLIS_DELAY_BEFORE_SLEEP 1000
#define DEEPSLEEP_MINUTES_AFTER_RENDER 6
// Image URL and jpg settings. Make sure to update WIDTH/HEIGHT if using loremflickr

// DISPLAY AND IMAGE CONFIG
#define EPD_WIDTH 1200
#define EPD_HEIGHT 825
#define IMG_URL "https://loremflickr.com/" STR(EPD_WIDTH) "/" STR(EPD_HEIGHT)

// Additionally you can try CALE.es to create a custom JPG gallery
// Using a non-ssl URL is faster since does not have to sync time
//#define IMG_URL ("http://img.cale.es/jpg/fasani/5e5ff140694ee")

// idf >= 4.3 needs VALIDATE_SSL_CERTIFICATE set to true for https URLs
// Please check the README to understand how to use an SSL Certificate
// Note: This makes a sntp time sync query for cert validation  (It's slower)
// IMPORTANT: idf updated and now when you use Internet requests you need to server cert
// verification
//            heading ESP-TLS in
//            https://newreleases.io/project/github/espressif/esp-idf/release/v4.3-beta1
#define VALIDATE_SSL_CERTIFICATE true
// To make an insecure request please check Readme

// As default is 512 without setting buffer_size property in esp_http_client_config_t
#define HTTP_RECEIVE_BUFFER_SIZE 1024

#define DEBUG_VERBOSE false
