
/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "socket_udp.h"

#define PORT CONFIG_EXAMPLE_PORT

static const char *UDP_TAG = "example";

 
/*********************************************************
 * Variables
 *********************************************************/
int sock;
struct sockaddr_in6 dest_addr;

/*********************************************************
 * Functions
 *********************************************************/
void udp_socket_init(void)
{
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    struct sockaddr_in local_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(3333),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };

    sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        return;
    }

    // Configura o socket para non-blocking
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    int err = bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr));
    if (err < 0) {
        close(sock);
        sock = -1;
        return;
    }
}

esp_err_t udp_socket_send(char *data, int len)
{
    socklen_t socklen = sizeof(dest_addr);

    if (sock < 0) {
        // ESP_LOGE(UDP_TAG, "Socket not created");
        return ESP_FAIL;
    }

    int err = sendto(sock, data, len, 0, (struct sockaddr *)&dest_addr, socklen);
    if (err < 0) {
        // ESP_LOGE(UDP_TAG, "Error occurred during sending: errno %d", errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

int udp_socket_receive(char *buffer, int buffer_len)
{
    struct sockaddr_storage source_addr;
    socklen_t socklen = sizeof(source_addr);

    int len = recvfrom(sock, buffer, buffer_len - 1, 0,
                       (struct sockaddr *)&source_addr, &socklen);
    if (len < 0) {
        // ESP_LOGE(UDP_TAG, "recvfrom failed: errno %d", errno);
        return -1;
    }

    buffer[len] = '\0';

    // Salva o IP/porta do remetente para usar no envio de resposta
    memcpy(&dest_addr, &source_addr, sizeof(dest_addr));

    // Força a porta de resposta para 3333, se desejar
    ((struct sockaddr_in *)&dest_addr)->sin_port = htons(3333);

    return len;
}


