#pragma once
/* Hard-coded waypoint list for bench + simulator testing.
 * Replace with the actual test-loop coordinates before on-vehicle runs.
 *
 * Current placeholder: a ~20 m square loop near the UW Bothell soccer field,
 * chosen so the points are clearly separated in GPS and easy to eyeball
 * on a map. Update to your real test site when available.
 */

struct Waypoint {
  double lat_deg;
  double lon_deg;
};

static const Waypoint waypoints[] = {
  { 47.7600,  -122.1917 },  // origin-ish
  { 47.7602,  -122.1917 },  // ~22 m north
  { 47.7602,  -122.1914 },  // ~22 m east
  { 47.7600,  -122.1914 },  // ~22 m south
};

static const int NUM_WAYPOINTS = sizeof(waypoints) / sizeof(waypoints[0]);
