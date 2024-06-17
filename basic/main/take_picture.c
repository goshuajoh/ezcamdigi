#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_tls.h"
#include "esp_http_client.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"

#define BOUNDARY "X-ESPIDF_MULTIPART"
#define SERVER "api.telegram.org"
#define PORT "443"
#define PATH "/bot7397170163:AAEyJvVdmEfe1Jt2hKoWH-KZZWu-OOIr56s/sendPhoto"
#define WIFI_SSID "Goshuajoh"
#define WIFI_PASS "Spidermanisc00l"

#define BOARD_LCD_MOSI 47
#define BOARD_LCD_MISO -1
#define BOARD_LCD_SCK 21
#define BOARD_LCD_CS 44
#define BOARD_LCD_DC 43
#define BOARD_LCD_RST -1
#define BOARD_LCD_BL 48
#define BOARD_LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define BOARD_LCD_BK_LIGHT_ON_LEVEL 0
#define BOARD_LCD_BK_LIGHT_OFF_LEVEL !BOARD_LCD_BK_LIGHT_ON_LEVEL
#define BOARD_LCD_H_RES 240
#define BOARD_LCD_V_RES 240
#define BOARD_LCD_CMD_BITS 8
#define BOARD_LCD_PARAM_BITS 8
#define LCD_HOST SPI2_HOST

#define CAMERA_MODULE_NAME "ESP-S3-EYE"
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET -1

#define CAMERA_PIN_VSYNC 6
#define CAMERA_PIN_HREF 7
#define CAMERA_PIN_PCLK 13
#define CAMERA_PIN_XCLK 15

#define CAMERA_PIN_SIOD 4
#define CAMERA_PIN_SIOC 5

#define CAMERA_PIN_D0 11
#define CAMERA_PIN_D1 9
#define CAMERA_PIN_D2 8
#define CAMERA_PIN_D3 10
#define CAMERA_PIN_D4 12
#define CAMERA_PIN_D5 18
#define CAMERA_PIN_D6 17
#define CAMERA_PIN_D7 16

#define BOOT_BUTTON_GPIO GPIO_NUM_0

#define EXAMPLE_MAX_CHAR_SIZE 64

#define EXAMPLE_ESP_WIFI_SSID "Goshuajoh"
#define EXAMPLE_ESP_WIFI_PASS "Spidermanisc00l"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN

extern const uint8_t telegram_certificate_pem_start[] asm("_binary_telegram_certificate_pem_start");
extern const uint8_t telegram_certificate_pem_end[] asm("_binary_telegram_certificate_pem_end");

volatile bool freeze_frame = false;

static const char *TAG = "digicam";
static esp_lcd_panel_handle_t panel_handle = NULL;
static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueFrameO = NULL;

camera_config_t camera_config = {
    .ledc_channel = LEDC_CHANNEL_0,
    .ledc_timer = LEDC_TIMER_0,
    .pin_d0 = CAMERA_PIN_D0,
    .pin_d1 = CAMERA_PIN_D1,
    .pin_d2 = CAMERA_PIN_D2,
    .pin_d3 = CAMERA_PIN_D3,
    .pin_d4 = CAMERA_PIN_D4,
    .pin_d5 = CAMERA_PIN_D5,
    .pin_d6 = CAMERA_PIN_D6,
    .pin_d7 = CAMERA_PIN_D7,
    .pin_xclk = CAMERA_PIN_XCLK,
    .pin_pclk = CAMERA_PIN_PCLK,
    .pin_vsync = CAMERA_PIN_VSYNC,
    .pin_href = CAMERA_PIN_HREF,
    .pin_sscb_sda = CAMERA_PIN_SIOD,
    .pin_sscb_scl = CAMERA_PIN_SIOC,
    .pin_pwdn = CAMERA_PIN_PWDN,
    .pin_reset = CAMERA_PIN_RESET,
    .fb_location = CAMERA_FB_IN_PSRAM,
    // .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .xclk_freq_hz = 16000000,
    .pixel_format = PIXFORMAT_RGB565, // PIXFORMAT_JPEG if using JPEG
    .frame_size = FRAMESIZE_240X240,  // FRAMESIZE_QVGA, FRAMESIZE_CIF, etc.
    .jpeg_quality = 6,                // 0-63 lower number means higher quality
    .fb_count = 1,                    // If more than one, i2s runs in continuous mode. Use only with JPEG
};

camera_config_t camera_config1 = {
    .ledc_channel = LEDC_CHANNEL_0,
    .ledc_timer = LEDC_TIMER_0,
    .pin_d0 = CAMERA_PIN_D0,
    .pin_d1 = CAMERA_PIN_D1,
    .pin_d2 = CAMERA_PIN_D2,
    .pin_d3 = CAMERA_PIN_D3,
    .pin_d4 = CAMERA_PIN_D4,
    .pin_d5 = CAMERA_PIN_D5,
    .pin_d6 = CAMERA_PIN_D6,
    .pin_d7 = CAMERA_PIN_D7,
    .pin_xclk = CAMERA_PIN_XCLK,
    .pin_pclk = CAMERA_PIN_PCLK,
    .pin_vsync = CAMERA_PIN_VSYNC,
    .pin_href = CAMERA_PIN_HREF,
    .pin_sscb_sda = CAMERA_PIN_SIOD,
    .pin_sscb_scl = CAMERA_PIN_SIOC,
    .pin_pwdn = CAMERA_PIN_PWDN,
    .pin_reset = CAMERA_PIN_RESET,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .xclk_freq_hz = 20000000,
    .pixel_format = PIXFORMAT_JPEG, // PIXFORMAT_JPEG if using JPEG
    .frame_size = FRAMESIZE_VGA,    // FRAMESIZE_QVGA, FRAMESIZE_CIF, etc.
    .jpeg_quality = 10,              // 0-63 lower number means higher quality
    .fb_count = 1,                  // If more than one, i2s runs in continuous mode. Use only with JPEG
};

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        if (!esp_http_client_is_chunked_response(evt->client))
        {
            ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            printf("%.*s", evt->data_len, (char *)evt->data);
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGI(TAG, "HTTP_EVENT_REDIRECT");
        break;
    }
    return ESP_OK;
}

void configure_gpio()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
}

static int s_retry_num = 0;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            ESP_LOGI(TAG, "connect to the AP fail");
            return;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void capture_and_send_image_task(void *pvParameters)
{
    freeze_frame = true;
    // Deinit camera first
    esp_camera_deinit();
    // Initialize with new configurations
    esp_camera_init(&camera_config1);
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);          // flip it back
    s->set_brightness(s, 1);     // -2 to 2
    s->set_contrast(s, -1);      // -2 to 2
    s->set_saturation(s, -2);    // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    // s->set_ae_level(s, 2); // -2 to 2
    // s->set_aec_value(s, 400); // 0 to 1200
    s->set_gain_ctrl(s, 0);                  // 0 = disable , 1 = enable
    s->set_agc_gain(s, 0);                   // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)6); // 0 to 6
    s->set_bpc(s, 1);                        // 0 = disable , 1 = enable
    s->set_wpc(s, 1);                        // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);                    // 0 = disable , 1 = enable (makes much lighter and noisy)
    s->set_lenc(s, 0);                       // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);                    // 0 = disable , 1 = enable
    // s->set_vflip(s, 0);                      // 0 = disable , 1 = enable
    s->set_dcw(s, 0);                        // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);                   // 0 = disable , 1 = enable
    s->set_reg(s, 0xff, 0xff, 0x01);         // banksel
    s->set_reg(s, 0x11, 0xff, 01);           // frame rate

    s->set_reg(s, 0xff, 0xff, 0x00); // banksel
    s->set_reg(s, 0x86, 0xff, 1);    // disable effects

    s->set_reg(s, 0xd3, 0xff, 5); // clock

    // s->set_reg(s, 0x42, 0xff, 0x4f); // image quality (lower is bad)
    // s->set_reg(s, 0x44, 0xff, 1);    // quality
    // Capture image from camera
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
        ESP_LOGE(TAG, "Camera capture failed");
        vTaskDelete(NULL);
        return;
    }
    // if (fb->format != PIXFORMAT_RGB565)
    // {
    //     ESP_LOGE(TAG, "Image format not JPEG");
    //     esp_camera_fb_return(fb);
    //     vTaskDelete(NULL);
    //     return;
    // }
    // uint8_t *jpeg_buf = NULL;
    // size_t jpeg_len = 0;
    uint8_t *jpeg_buf = fb->buf;
    size_t jpeg_len = fb->len;
    // if (!frame2jpg(fb, 60, &jpeg_buf, &jpeg_len))
    // {
    //     ESP_LOGE(TAG, "JPEG compression failed");
    //     esp_camera_fb_return(fb);
    //     vTaskDelete(NULL);
    //     return;
    // }

    char url[512];
    snprintf(url, sizeof(url), "https://%s%s", SERVER, PATH);

    char boundary[] = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
    char content_type[128];
    snprintf(content_type, sizeof(content_type), "multipart/form-data; boundary=%s", boundary);

    char *chat_id = "219745533";
    // char *chat_id = "646462341";

    // Start of the form data
    char form_data_start[512];
    snprintf(form_data_start, sizeof(form_data_start),
             "--%s\r\n"
             "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n"
             "%s\r\n"
             "--%s\r\n"
             "Content-Disposition: form-data; name=\"photo\"; filename=\"image.jpg\"\r\n"
             "Content-Type: image/jpeg\r\n\r\n",
             boundary, chat_id, boundary);

    // End of the form data
    char form_data_end[128];
    snprintf(form_data_end, sizeof(form_data_end), "\r\n--%s--\r\n", boundary);

    // Calculate total length for content-length header
    int total_len = strlen(form_data_start) + jpeg_len + strlen(form_data_end);
    char content_length[16];
    snprintf(content_length, sizeof(content_length), "%d", total_len);

    esp_http_client_config_t config = {
        .url = url,
        .event_handler = _http_event_handler,
        .cert_pem = (const char *)telegram_certificate_pem_start,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", content_type);
    esp_http_client_set_header(client, "Content-Length", content_length);

    esp_err_t err = esp_http_client_open(client, total_len);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        esp_camera_fb_return(fb);
        vTaskDelete(NULL);
        return;
    }

    // Write the form data start
    esp_http_client_write(client, form_data_start, strlen(form_data_start));
    // Write the image data
    esp_http_client_write(client, (const char *)jpeg_buf, jpeg_len);
    // Write the form data end
    esp_http_client_write(client, form_data_end, strlen(form_data_end));

    char recv_buf[1024];
    int response_len = esp_http_client_fetch_headers(client);
    response_len = esp_http_client_read_response(client, recv_buf, sizeof(recv_buf));
    if (response_len >= 0)
    {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %lld",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
        ESP_LOGI(TAG, "HTTP Response: %.*s", response_len, recv_buf);
    }
    else
    {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    esp_camera_fb_return(fb);
    esp_camera_deinit();
    esp_camera_init(&camera_config);
    s->set_vflip(s, 1); // flip it back
    vTaskDelete(NULL);
    vTaskDelay(1000);
    freeze_frame = false;
}

void capture_and_save_image()
{
    esp_camera_deinit();
    esp_camera_init(&camera_config1);
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
        ESP_LOGE(TAG, "Camera capture failed");
        return;
    }
    else
    {
        if (fb->format == PIXFORMAT_JPEG)
        {
            ESP_LOGI(TAG, "we did it!");
        }
    }
    // Create a unique file name
    // char path[32];
    // static int picture_count = 0;
    // snprintf(path, sizeof(path), MOUNT_POINT "/picture_%03d.jpg", picture_count++);

    // const char *path = MOUNT_POINT "/hello.jpg";

    // ESP_LOGI(TAG, "Saving to path: %s", path);
    // FILE *file = fopen(path, "w");
    // if (file != NULL)
    // {
    //     size_t written = fwrite(fb->buf, 1, fb->len, file);
    //     if (written != fb->len)
    //     {
    //         ESP_LOGE(TAG, "Failed to write complete file. Written: %d, Expected: %d", written, fb->len);
    //     }
    //     else
    //     {
    //         ESP_LOGI(TAG, "File saved: %s", path);
    //     }
    //     fclose(file);
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "Failed to open file in write mode: %s", strerror(errno));
    // }

    // Return the frame buffer promptly to avoid overflow
    esp_camera_fb_return(fb);
    esp_camera_deinit();
    esp_camera_init(&camera_config);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    wifi_init_sta();
    configure_gpio();
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1); // flip it back
    void app_lcd_set_color(int color)
    {
        uint16_t *buffer = (uint16_t *)malloc(BOARD_LCD_H_RES * sizeof(uint16_t));
        if (NULL == buffer)
        {
            ESP_LOGE(TAG, "Memory for bitmap is not enough");
        }
        else
        {
            for (size_t i = 0; i < BOARD_LCD_H_RES; i++)
            {
                buffer[i] = color;
            }

            for (int y = 0; y < BOARD_LCD_V_RES; y++)
            {
                esp_lcd_panel_draw_bitmap(panel_handle, 0, y, BOARD_LCD_H_RES, y + 1, buffer);
            }

            free(buffer);
        }
    }

    esp_err_t register_lcd(const QueueHandle_t frame_i, const QueueHandle_t frame_o, const bool return_fb)
    {
        ESP_LOGI(TAG, "Initialize SPI bus");
        spi_bus_config_t bus_conf = {
            .sclk_io_num = BOARD_LCD_SCK,
            .mosi_io_num = BOARD_LCD_MOSI,
            .miso_io_num = BOARD_LCD_MISO,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = BOARD_LCD_H_RES * BOARD_LCD_V_RES * sizeof(uint16_t),
        };
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_conf, SPI_DMA_CH_AUTO));

        ESP_LOGI(TAG, "Install panel IO");
        esp_lcd_panel_io_handle_t io_handle = NULL;
        esp_lcd_panel_io_spi_config_t io_config = {
            .dc_gpio_num = BOARD_LCD_DC,
            .cs_gpio_num = BOARD_LCD_CS,
            .pclk_hz = BOARD_LCD_PIXEL_CLOCK_HZ,
            .lcd_cmd_bits = BOARD_LCD_CMD_BITS,
            .lcd_param_bits = BOARD_LCD_PARAM_BITS,
            .spi_mode = 0,
            .trans_queue_depth = 10,
        };
        // Attach the LCD to the SPI bus
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));

        // ESP_LOGI(TAG, "Install ST7789 panel driver");
        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = BOARD_LCD_RST,
            .rgb_endian = LCD_RGB_ENDIAN_RGB,
            .bits_per_pixel = 16,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
        esp_lcd_panel_invert_color(panel_handle, true); // Set inversion for esp32s3eye

        // turn on display
        esp_lcd_panel_disp_on_off(panel_handle, true);

        app_lcd_set_color(0x000000);

        return ESP_OK;
    }
    register_lcd(xQueueFrameI, xQueueFrameO, true);
    // Turn on backlight
    gpio_set_direction(BOARD_LCD_BL, GPIO_MODE_OUTPUT);
    gpio_set_level(BOARD_LCD_BL, BOARD_LCD_BK_LIGHT_ON_LEVEL);

    // Stream the camera feed to the LCD
    while (true)
    {
        if (gpio_get_level(BOOT_BUTTON_GPIO) == 0)
        {
            ESP_LOGI(TAG, "Button Pressed? HUHHH");
            // Debounce delay
            vTaskDelay(10 / portTICK_PERIOD_MS);
            if (gpio_get_level(BOOT_BUTTON_GPIO) == 0)
            {
                // Capture and save image
                xTaskCreate(&capture_and_send_image_task, "capture_and_send_image_task", 1* 8192, NULL, 5, NULL);
                // test_sd_card_write_small_chunks();

                // Wait for button release
                while (gpio_get_level(BOOT_BUTTON_GPIO) == 0)
                {
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay to prevent CPU overload
        if (freeze_frame == false)
        {
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb)
            {
                ESP_LOGE(TAG, "Camera Capture Failed");
                continue;
            }

            err = esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, BOARD_LCD_H_RES, BOARD_LCD_V_RES, fb->buf);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "LCD Panel Draw Bitmap Failed");
            }

            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(10)); // Adjust delay as needed
        }
    }
}
