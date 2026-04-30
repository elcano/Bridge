#pragma once
/* Flat-earth Euclidean navigation math, cm-frame.
 *
 * Per project convention (see Common.cpp in elcano/HighLevel and
 * elcanoproject.org/wiki/Communication), the system operates in a small
 * area where the map is locally flat. Lat/lon is only used at the system
 * boundary (e.g., for the origin in CAN message 0x251 SetOrigin); all
 * other location math is done in a Cartesian (east_cm, north_cm) frame
 * relative to the origin.
 *
 * Integer types are preferred. Trig calls use double, but downstream
 * arithmetic stays in int32 cm and centidegree units.
 */
#include <stdint.h>

struct Origin {
  double lat;       // degrees
  double lon;       // degrees
  double cos_lat;   // pre-computed cos(lat * TO_RADIANS)
};

// Initialize an origin (call once at startup).
void origin_init(Origin& o, double lat_deg, double lon_deg);

// Convert lat/lon (degrees) to local cm-frame east/north (int32).
void latlon_to_cm(const Origin& o, double lat_deg, double lon_deg,
                  int32_t& east_cm, int32_t& north_cm);

// Pythagorean distance in cm between two points in the local frame.
int32_t distance_cm(int32_t e1, int32_t n1, int32_t e2, int32_t n2);

// Bearing from (e1,n1) to (e2,n2) in centidegrees, range [0, 35999].
// 0 = north, 9000 = east, 18000 = south, 27000 = west.
int32_t bearing_centiDeg(int32_t e1, int32_t n1, int32_t e2, int32_t n2);

// Wrap an angle in centidegrees to (-18000, +18000].
int32_t wrap_centiDeg_signed(int32_t x);
