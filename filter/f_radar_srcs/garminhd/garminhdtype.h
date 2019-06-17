#ifdef INITIALIZE_RADAR

static const NetworkAddress g_data(239, 254, 2, 0, 50100);
static const NetworkAddress g_report(239, 254, 2, 0, 50100);
static const NetworkAddress g_send(172, 16, 2, 0, 50101);

#endif

#define RANGE_METRIC_RT_GARMIN_HD \
  { 250, 500, 750, 1000, 1500, 2000, 3000, 4000, 6000, 8000, 12000, 16000, 24000, 36000, 48000, 64000 }
// Garmin mixed range is the same as nautical miles, it does not support really short ranges
#define RANGE_MIXED_RT_GARMIN_HD                                                                                          \
  {                                                                                                                       \
    1852 / 8, 1852 / 4, 1852 / 2, 1852 * 3 / 4, 1852 * 1, 1852 * 3 / 2, 1852 * 2, 1852 * 3, 1852 * 4, 1852 * 6, 1852 * 8, \
        1852 * 12, 1852 * 16, 1852 * 24, 1852 * 36, 1852 * 48                                                             \
  }
#define RANGE_NAUTIC_RT_GARMIN_HD                                                                                         \
  {                                                                                                                       \
    1852 / 8, 1852 / 4, 1852 / 2, 1852 * 3 / 4, 1852 * 1, 1852 * 3 / 2, 1852 * 2, 1852 * 3, 1852 * 4, 1852 * 6, 1852 * 8, \
        1852 * 12, 1852 * 16, 1852 * 24, 1852 * 36, 1852 * 48                                                             \
  }

// Garmin HD has 720 spokes of varying 519 - 705 bytes each
#define GARMIN_HD_SPOKES 720
#define GARMIN_HD_MAX_SPOKE_LEN 705

#if SPOKES_MAX < GARMIN_HD_SPOKES
#undef SPOKES_MAX
#define SPOKES_MAX GARMIN_HD_SPOKES
#endif
#if SPOKE_LEN_MAX < GARMIN_HD_MAX_SPOKE_LEN
#undef SPOKE_LEN_MAX
#define SPOKE_LEN_MAX GARMIN_HD_MAX_SPOKE_LEN
#endif

DEFINE_RADAR(RT_GARMIN_HD,                              /* Type */
             "Garmin HD",                          /* Name */
             GARMIN_HD_SPOKES,                          /* Spokes */
             GARMIN_HD_MAX_SPOKE_LEN,                   /* Spoke length */
             GarminHDControlsDialog,                    /* Controls class */
             GarminHDReceive(g_report, g_data), /* Receive class */
             GarminHDControl(g_send),                   /* Send/Control class */
             0                                          /* 0 = Primary, 1 = Secondary etc */
)
