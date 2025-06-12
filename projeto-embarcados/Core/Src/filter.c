/*
 * filter.c
 *
 *  Created on: Jun 12, 2025
 *      Author: FeruMaga
 */
#include "filter.h"

float Filter_Lowpass(float oldValue, float newValue, float alpha) {
    return (alpha * newValue) + ((1.0f - alpha) * oldValue);
}
