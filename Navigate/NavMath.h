#pragma once
/* Pure-function geodesy helpers for waypoint navigation.
 * No Arduino dependencies — the intent is to port these to the Jetson Nano
 * later, so keep globals out and use only <math.h>.
 */

// Great-circle distance between two lat/lon points, in meters.
// Inputs are decimal degrees (e.g. 47.6553, -122.3035).
double haversine_m(double lat1_deg, double lon1_deg,
                   double lat2_deg, double lon2_deg);

// Initial great-circle bearing from point 1 to point 2, in degrees [0, 360).
// 0 = north, 90 = east. Inputs are decimal degrees.
double bearing_deg(double lat1_deg, double lon1_deg,
                   double lat2_deg, double lon2_deg);

// Wrap an angle (degrees) into the range (-180, 180].
double angleWrap180(double deg);
