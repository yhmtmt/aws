#ifndef GARMINXHD_H
#define GARMINXHD_H

#define DEGREES_PER_ROTATION (360)
#define WATCHDOG_TIMEOUT (10)  // After 10s assume GPS and heading data is invalid
#define DATA_TIMEOUT (5)
#define TIMED_OUT(t, timeout) (t >= timeout)


struct GeoPosition {
  double lat;
  double lon;
};

enum RadarState {
  RADAR_OFF,
  RADAR_STANDBY,
  RADAR_WARMING_UP,
  RADAR_TIMED_IDLE,
  RADAR_STOPPING,
  RADAR_SPINNING_DOWN,
  RADAR_STARTING,
  RADAR_SPINNING_UP,
  RADAR_TRANSMIT
};


typedef enum ControlType {
  CT_NONE,
#define CONTROL_TYPE(x, y) x,
#include "ControlType.inc"
#undef CONTROL_TYPE
  CT_MAX
} ControlType;


enum RangeUnits { RANGE_MIXED, RANGE_METRIC, RANGE_NAUTIC };
static const int RangeUnitsToMeters[3] = {1852, 1000, 1852};

static const NetworkAddress gx_data(239, 254, 2, 0, 50102);
static const NetworkAddress gx_report(239, 254, 2, 0, 50100);
static const NetworkAddress gx_send(172, 16, 2, 0, 50101);
#define RANGE_METRIC_RT_GARMIN_XHD \
  { 250, 500, 750, 1000, 1500, 2000, 3000, 4000, 6000, 8000, 12000, 16000, 24000, 36000, 48000, 64000 }
// Garmin mixed range is the same as nautical miles, it does not support really short ranges
#define RANGE_MIXED_RT_GARMIN_XHD                                                                                         \
  {                                                                                                                       \
    1852 / 8, 1852 / 4, 1852 / 2, 1852 * 3 / 4, 1852 * 1, 1852 * 3 / 2, 1852 * 2, 1852 * 3, 1852 * 4, 1852 * 6, 1852 * 8, \
        1852 * 12, 1852 * 16, 1852 * 24, 1852 * 36, 1852 * 48                                                             \
  }
#define RANGE_NAUTIC_RT_GARMIN_XHD                                                                                        \
  {                                                                                                                       \
    1852 / 8, 1852 / 4, 1852 / 2, 1852 * 3 / 4, 1852 * 1, 1852 * 3 / 2, 1852 * 2, 1852 * 3, 1852 * 4, 1852 * 6, 1852 * 8, \
        1852 * 12, 1852 * 16, 1852 * 24, 1852 * 36, 1852 * 48                                                             \
  }

// Garmin xHD has 1440 spokes of varying 519 - 705 bytes each
#define GARMIN_XHD_SPOKES 1440
#define GARMIN_XHD_MAX_SPOKE_LEN 705

#endif
