//
// Created by Kurosu Chan on 2024/1/18.
//

#ifndef FIXED_POINT_H
#define FIXED_POINT_H

#ifdef APP_USE_FIXED_POINT
#include <cnl/scaled_integer.h>

using fixed_16_16   = cnl::scaled_integer<uint32_t, cnl::power<-16>>;
using s_fixed_16_16 = cnl::scaled_integer<int32_t, cnl::power<-16>>;
using fixed_24_8    = cnl::scaled_integer<uint32_t, cnl::power<-8>>;
using fixed_8_8     = cnl::scaled_integer<uint16_t, cnl::power<-8>>;
#else
using fixed_16_16   = float;
using s_fixed_16_16 = float;
using fixed_24_8    = float;
using fixed_8_8     = float;
#endif // APP_USE_FIXED_POINT

#endif // FIXED_POINT_H
