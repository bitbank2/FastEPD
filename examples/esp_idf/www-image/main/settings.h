// WiFi configuration:
#define ESP_WIFI_SSID ""
#define ESP_WIFI_PASSWORD "yourPass"

// DISPLAY settings
#define EPD_WIDTH 1200
#define EPD_HEIGHT 825
#define EPD_ROTATION 0   // 0, 90, 180
// Affects the gamma to calculate gray (lower is darker/higher contrast)
// Nice test values: 0.9 1.2 1.4 higher and is too bright
double gamma_value = 0.9;

// Deepsleep configuration
#define MILLIS_DELAY_BEFORE_SLEEP 1000
#define DEEPSLEEP_MINUTES_AFTER_RENDER 6

// Image URL and jpg settings. Make sure to update WIDTH/HEIGHT if using loremflickr
// Note: Only HTTP protocol supported (Check README to use SSL secure URLs) loremflickr
//#define IMG_URL ("https://picsum.photos/1200/825")
#define IMG_URL ("http://img.cale.es/jpg/fasani/5e5ff140694ee")
// idf >= 4.3 needs VALIDATE_SSL_CERTIFICATE set to true for https URLs
// Please check the README to understand how to use an SSL Certificate
// Note: This makes a sntp time sync query for cert validation  (It's slower)
// IMPORTANT: idf updated and now when you use Internet requests you need to server cert
// verification
//            heading ESP-TLS in
//            https://newreleases.io/project/github/espressif/esp-idf/release/v4.3-beta1
#define VALIDATE_SSL_CERTIFICATE false
// To make an insecure request please check Readme

// As default is 512 without setting buffer_size property in esp_http_client_config_t
#define HTTP_RECEIVE_BUFFER_SIZE 1986

#define DEBUG_VERBOSE true
