#include "helper_functions.h"

float clamp(float x, float min, float max)
{
    x < min ? x = min : x;
    x > max ? x = max : x;

    return x;
}