#pragma once
/* Sensor Hub / High-Level Due per-vehicle settings template.
 * Copy this file to Settings.h (git-ignored) and tune for the specific trike.
 * Keep SettingsTemplate.h in source control; Settings.h should NOT be committed.
 */

// ---- Navigation tunables ----
// Cruise speed commanded to DBW while following waypoints (cm/s).
#define CRUISE_CMPS 100

// Proportional gain for heading-error -> steering angle.
// Units: DegX10 of commanded steering per degree of heading error.
#define Kp_STEERING 5

// Distance (cm) within which a waypoint is considered reached.
#define WAYPOINT_RADIUS_CM 300

// Maximum commanded steering magnitude in whole degrees (absolute cap).
// The 0x350 angle field is DegX10, so the wire value is clamped to +/- (MAX_STEER_DEG * 10).
#define MAX_STEER_DEG 30

// Main loop rate (Hz). Controls 0x350/0x100 emission rate.
#define NAV_LOOP_HZ 10
