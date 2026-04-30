#include "NavMath.h"
#include <math.h>

static const double EARTH_RADIUS_M = 6371000.0;
static const double DEG2RAD = M_PI / 180.0;
static const double RAD2DEG = 180.0 / M_PI;

double haversine_m(double lat1_deg, double lon1_deg,
                   double lat2_deg, double lon2_deg) {
  double lat1 = lat1_deg * DEG2RAD;
  double lat2 = lat2_deg * DEG2RAD;
  double dlat = (lat2_deg - lat1_deg) * DEG2RAD;
  double dlon = (lon2_deg - lon1_deg) * DEG2RAD;
  double a = sin(dlat / 2) * sin(dlat / 2) +
             cos(lat1) * cos(lat2) * sin(dlon / 2) * sin(dlon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return EARTH_RADIUS_M * c;
}

double bearing_deg(double lat1_deg, double lon1_deg,
                   double lat2_deg, double lon2_deg) {
  double lat1 = lat1_deg * DEG2RAD;
  double lat2 = lat2_deg * DEG2RAD;
  double dlon = (lon2_deg - lon1_deg) * DEG2RAD;
  double y = sin(dlon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
  double brng = atan2(y, x) * RAD2DEG;
  if (brng < 0) brng += 360.0;
  return brng;
}

double angleWrap180(double deg) {
  while (deg > 180.0)  deg -= 360.0;
  while (deg <= -180.0) deg += 360.0;
  return deg;
}
