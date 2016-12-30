#include <string.h>
#include <stdio.h>

#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "lwip/api.h"
#include "ssid_config.h"
#include "esp8266.h"
#include "ws2812.h"

#define GPIO_A 5
#define GPIO_B 4
#define GPIO_SELECT 14
#define GPIO_START 15
#define GPIO_UP 13
#define GPIO_DOWN 2
#define GPIO_LEFT 12
#define GPIO_RIGHT 0
#define GPIO_LED 3
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"

static QueueHandle_t buttonqueue;
static QueueHandle_t ledqueue;
static uint8_t hwaddr[6];

void udp_server(void *pvParameters)
{
    QueueHandle_t *ledqueue = (QueueHandle_t *)pvParameters;
    printf("Starting UDP Server\n");
    while(1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void led_controller(void *pvParameters)
{
    QueueHandle_t *ledqueue = (QueueHandle_t *)pvParameters;
    printf("Turning on LEDs\n");
    ws2812_seq_start();
    ws2812_seq_rgb(GPIO_LED, 0x0F0000);
    ws2812_seq_rgb(GPIO_LED, 0x000F00);
    ws2812_seq_rgb(GPIO_LED, 0x00000F);
    ws2812_seq_rgb(GPIO_LED, 0x0F0F00);
    ws2812_seq_end();
    printf("LEDs should be on\n");
    while(1)
    {
        printf("Looping\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void button_watcher(void *pvParameters)
{
    QueueHandle_t *buttonqueue = (QueueHandle_t *)pvParameters;
    printf("Starting Button Watcher\n");
    // Broadcaster part
    err_t err;

    while(1) {

        // Send out some UDP data
        struct netconn* conn;

        printf("Starting UDP Connection\n");
        // Create UDP connection
        conn = netconn_new(NETCONN_UDP);

        printf("Binding to local port\n");
        // Connect to local port
        err = netconn_bind(conn, IP_ADDR_ANY, 8004);

        if (err != ERR_OK) {
            netconn_delete(conn);
            printf("%s : Could not bind! (%s)\n", __FUNCTION__, lwip_strerr(err));
            continue;
        }

        printf("Creating broadcast socket on port 8005\n");
        err = netconn_connect(conn, IP_ADDR_BROADCAST, 8005);

        if (err != ERR_OK) {
            netconn_delete(conn);
            printf("%s : Could not connect! (%s)\n", __FUNCTION__, lwip_strerr(err));
            continue;
        }

        for(;;) {
            uint8_t button;
            printf("Waiting for queue message\n");
            xQueueReceive(*buttonqueue, &button, portMAX_DELAY);
            printf("Button %d pushed.\n", button);
            struct netbuf* buf = netbuf_new();
            void* data = netbuf_alloc(buf, 7);
            memcpy(buf, hwaddr, 6);
            memcpy(buf+6,button, 1);
            err = netconn_send(conn, buf);

            if (err != ERR_OK) {
                printf("%s : Could not send data!!! (%s)\n", __FUNCTION__, lwip_strerr(err));
                continue;
            }
            netbuf_delete(buf); // De-allocate packet buffer
        }

        err = netconn_disconnect(conn);
        printf("%s : Disconnected from IP_ADDR_BROADCAST port 12346 (%s)\n", __FUNCTION__, lwip_strerr(err));

        err = netconn_delete(conn);
        printf("%s : Deleted connection (%s)\n", __FUNCTION__, lwip_strerr(err));

        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void gpio00_interrupt_handler(void)
{
    printf("RIGHT\n");
    xQueueSendToBackFromISR(buttonqueue, 0, NULL);
}       

void gpio02_interrupt_handler(void)
{
    printf("DOWN\n");
    //xQueueSendToBackFromISR(buttonqueue, 1, NULL);
}       

void gpio04_interrupt_handler(void)
{
    printf("B\n");
    //xQueueSendToBackFromISR(buttonqueue, 2, NULL);
}       

void gpio05_interrupt_handler(void)
{
    printf("A\n");
    //xQueueSendToBackFromISR(buttonqueue, 2, NULL);
}       

void gpio12_interrupt_handler(void)
{
    printf("LEFT\n");
    //xQueueSendToBackFromISR(buttonqueue, 2, NULL);
}       

void gpio13_interrupt_handler(void)
{
    printf("UP\n");
    //xQueueSendToBackFromISR(buttonqueue, 2, NULL);
}       

void gpio14_interrupt_handler(void)
{
    printf("SELECT\n");
    //xQueueSendToBackFromISR(buttonqueue, 2, NULL);
}       

void gpio15_interrupt_handler(void)
{
    printf("START\n");
    //xQueueSendToBackFromISR(buttonqueue, 2, NULL);
}       

void user_init(void)
{
    uart_set_baud(0, 115200);

    printf("SDK version:%s\n", sdk_system_get_sdk_version());
    printf("Welcome to your very own MAGBadge!\n");

    sdk_wifi_get_macaddr(STATION_IF, hwaddr);
    printf("Your MAC Address is " MACSTR "\n", MAC2STR(hwaddr));

    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

    // Required to call wifi_set_opmode before station_set_config.
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);

    gpio_enable(GPIO_LED, GPIO_OUTPUT);

    gpio_enable(GPIO_UP,     GPIO_INPUT);
    gpio_enable(GPIO_DOWN,   GPIO_INPUT);
    gpio_enable(GPIO_LEFT,   GPIO_INPUT);
    gpio_enable(GPIO_RIGHT,  GPIO_INPUT);
    gpio_enable(GPIO_START,  GPIO_INPUT);
    gpio_enable(GPIO_SELECT, GPIO_INPUT);
    gpio_enable(GPIO_B,      GPIO_INPUT);
    gpio_enable(GPIO_A,      GPIO_INPUT);

    gpio_set_interrupt(GPIO_UP,     GPIO_INTTYPE_EDGE_NEG);
    gpio_set_interrupt(GPIO_DOWN,   GPIO_INTTYPE_EDGE_NEG);
    gpio_set_interrupt(GPIO_LEFT,   GPIO_INTTYPE_EDGE_NEG);
    gpio_set_interrupt(GPIO_RIGHT,  GPIO_INTTYPE_EDGE_NEG);
    gpio_set_interrupt(GPIO_START,  GPIO_INTTYPE_EDGE_NEG);
    gpio_set_interrupt(GPIO_SELECT, GPIO_INTTYPE_EDGE_NEG);
    gpio_set_interrupt(GPIO_B,      GPIO_INTTYPE_EDGE_NEG);
    gpio_set_interrupt(GPIO_A,      GPIO_INTTYPE_EDGE_NEG);

    buttonqueue = xQueueCreate(16, 1);
    ledqueue = xQueueCreate(16, 4);
    xTaskCreate(&button_watcher, "Button Watcher", 256, &buttonqueue, 2, NULL);
    xTaskCreate(&led_controller, "LED Controller", 256, &ledqueue, 2, NULL);
    xTaskCreate(&udp_server, "UDP Server", 256, &ledqueue, 2, NULL);    
}
