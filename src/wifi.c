#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
// #include "lwip/netif.h"
#include "wifi.h"
#define WIFI_USER ""
#define WIFI_PASS ""

#define PORT 4242



static inline int wifi_connect(){
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return 1;

    }
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_USER, WIFI_PASS,
                                           CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Wi-Fi connect failed\n");
        return 1;
    }
    // printf("Connected! IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));
    return 0;
}
