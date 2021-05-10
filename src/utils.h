//
// Created by Kunal Sheth on 5/2/21.
//

#ifndef UNICYCLE_UTILS_H
#define UNICYCLE_UTILS_H

#include <stdint.h>

inline int64_t min(int64_t a, int64_t b) {
    return a < b ? a : b;
}

inline int64_t max(int64_t a, int64_t b) {
    return a > b ? a : b;
}

#endif //UNICYCLE_UTILS_H
