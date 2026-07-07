#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

namespace biosense
{
    class UdpSender {
        public:
            esp_err_t init(const char* ip, uint16_t port);
            esp_err_t send(const char* data, size_t len);

        private:
            int socket_fd_ = -1;
            struct sockaddr_in dest_addr_ {};
    };
} // namespace biosense