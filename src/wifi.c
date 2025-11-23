#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "lwip/netif.h"
#include "lwip/udp.h"
#include "wifi.h"
#define WIFI_USER ""
#define WIFI_PASS ""

#define PORT 4242


// used AI
int wifi_connect(){
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
    printf("Connected! IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));
    return 0;
}
void udp_receive_callback(void *arg,
                                 struct udp_pcb *pcb,
                                 struct pbuf *p,
                                 const ip_addr_t *addr,
                                 u16_t port)
{
    if (!p) return;

    printf("RX from %s:%d -> %.*s\n",
           ipaddr_ntoa(addr), port, p->len, (char*)p->payload);

    pbuf_free(p);
}

static inline void wifi_start_udp_server(uint16_t listen_port)
{
    struct udp_pcb *pcb = udp_new();

    if (!pcb) {
        printf("Failed to create UDP PCB\n");
        return;
    }

    udp_bind(pcb, IP_ADDR_ANY, listen_port);
    udp_recv(pcb, udp_receive_callback, NULL);

    printf("UDP listening on port %d\n", listen_port);
}