#pragma once

#include "esp_err.h"

namespace biosense {

    class WifiSta {
        public:
            esp_err_t connect();
    };

} 