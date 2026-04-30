#pragma once
/* Simulator -> Sensor Hub USB protocol constants.
 *
 * Router Arduino (or PC simulator) emits framed messages over Native USB.
 * Each frame starts with a header byte identifying the payload type;
 * payload format is per-message-type and documented below.
 *
 * This mirrors Minhee's updated Router-Arduino simulator so the HL Due
 * stays source-compatible once the full sim lands.
 */

#define SIM_HDR_COMPASS 0x05  // heading in deci-degrees (int16, 0..3599)
#define SIM_HDR_GPS     0x06  // lat/lon as NMEA-style ddmm.ssss text + hemisphere
#define SIM_HDR_VEL     0x07  // speed in cm/s (int16)

// Convert NMEA-style ddmm.ssss (e.g. 4745.6553) to decimal degrees.
// hemisphere: 'N' or 'S' for latitude, 'E' or 'W' for longitude.
// Returns NAN on malformed input.
static inline double parseGpsDdmmssss(double ddmm, char hemisphere) {
  if (ddmm < 0) return NAN;
  double deg = (int)(ddmm / 100);
  double min = ddmm - deg * 100;
  double decimal = deg + min / 60.0;
  if (hemisphere == 'S' || hemisphere == 'W') decimal = -decimal;
  return decimal;
}
