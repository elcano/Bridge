#include "NavMath.h"
#include <math.h>

// Earth radius in cm. The wire format (0x4C0 VehiclePosition) uses int32 cm,
// so cm-frame storage matches the protocol directly.
static const int32_t EARTH_RADIUS_CM = 637100000L;
static const double  TO_RADIANS      = 3.1415926535 / 180.0;

void origin_init(Origin& o, double lat_deg, double lon_deg) {
  o.lat = lat_deg;
  o.lon = lon_deg;
  o.cos_lat = cos(lat_deg * TO_RADIANS);
}

void latlon_to_cm(const Origin& o, double lat_deg, double lon_deg,
                  int32_t& east_cm, int32_t& north_cm) {
  double dLat = lat_deg - o.lat;
  double dLon = lon_deg - o.lon;
  double n = dLat * TO_RADIANS * (double)EARTH_RADIUS_CM;
  double e = dLon * TO_RADIANS * (double)EARTH_RADIUS_CM * o.cos_lat;
  north_cm = (int32_t)n;
  east_cm  = (int32_t)e;
}

int32_t distance_cm(int32_t e1, int32_t n1, int32_t e2, int32_t n2) {
  int64_t dE = (int64_t)e1 - (int64_t)e2;
  int64_t dN = (int64_t)n1 - (int64_t)n2;
  int64_t sq = dE * dE + dN * dN;
  return (int32_t)sqrt((double)sq);
}

int32_t bearing_centiDeg(int32_t e1, int32_t n1, int32_t e2, int32_t n2) {
  double dE = (double)(e2 - e1);
  double dN = (double)(n2 - n1);
  // atan2(east, north): 0 = pointing north, positive clockwise.
  double rad = atan2(dE, dN);
  double deg = rad / TO_RADIANS;
  if (deg < 0) deg += 360.0;
  return (int32_t)(deg * 100.0);
}

int32_t wrap_centiDeg_signed(int32_t x) {
  while (x >  18000) x -= 36000;
  while (x <= -18000) x += 36000;
  return x;
}
