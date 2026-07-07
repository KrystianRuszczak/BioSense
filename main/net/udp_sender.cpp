#include "udp_sender.hpp"

#include <cerrno>

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "esp_log.h"

namespace biosense
{

static const char* TAG = "UdpSender";

esp_err_t UdpSender::init(const char* ip, uint16_t port)
{
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    if (socket_fd_ < 0)
    {
        ESP_LOGE(TAG, "socket() failed, errno=%d", errno);
        return ESP_FAIL;
    }
    
    dest_addr_ = {};

    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port = htons(port);

    int result = inet_pton(AF_INET, ip, &dest_addr_.sin_addr);
    if (result != 1)
    {
        ESP_LOGE(TAG, "Invalid IP address: %s", ip);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Target: %s:%u", ip, static_cast<unsigned>(port));

    return ESP_OK;
}

esp_err_t UdpSender::send(const char* data, size_t len)
{
    if (socket_fd_ < 0)
    {
        return ESP_ERR_INVALID_STATE;
    }
    
    int sent = sendto(
        socket_fd_,
        data,
        len,
        0,
        reinterpret_cast<sockaddr*>(&dest_addr_),
        sizeof(dest_addr_)
    );

    if (sent < 0)
    {
        ESP_LOGW(TAG, "sendto() failed, errno=%d", errno);
        return ESP_FAIL;
    }

    return ESP_OK;
}

} // namespace biosense
