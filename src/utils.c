#include "utils.h"
#include <math.h>


float getPitch(int16_t x, int16_t z) {
  return atan2(x, z) * 180.0 / M_PI; // Convert from radians to degrees
}
