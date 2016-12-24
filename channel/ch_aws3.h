#ifndef _CH_AWS3_H_
#define _CH_AWS3_H_
#include "ch_base.h"

// Copyright(c) 2016 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// ch_aws3.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_aws3.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_aws3.h.  If not, see <http://www.gnu.org/licenses/>. 
#include <mavlink.h>

class f_aws3_com;

struct s_aws3_param{
	int id;
	const char * str;
	const char * exp;
	union {
		unsigned char * uc;
		unsigned short * us;
		unsigned int * ui;
		char * c;
		short * s;
		int * i;
		float * f;
		double * d;
	};
	uint8_t type;

	s_aws3_param() : id(-1), str(NULL), exp(NULL), c(NULL)
	{};
	s_aws3_param(int _id, const char * _str, const char * _exp, unsigned char * _uc) :id(_id), str(_str), exp(_exp), type(MAV_PARAM_TYPE_UINT8), uc(_uc), key(-1), sync(false)
	{
	}
	s_aws3_param(int _id, const char * _str, const char * _exp, unsigned short * _us) :id(_id), str(_str), exp(_exp), type(MAV_PARAM_TYPE_UINT16), us(_us), key(-1), sync(false)
	{
	}
	s_aws3_param(int _id, const char * _str, const char * _exp, unsigned int * _ui) :id(_id), str(_str), exp(_exp), type(MAV_PARAM_TYPE_UINT32), ui(_ui), key(-1), sync(false)
	{
	}

	s_aws3_param(int _id, const char * _str, const char * _exp, char * _c) :id(_id), str(_str), exp(_exp), type(MAV_PARAM_TYPE_INT8), c(_c), key(-1), sync(false)
	{
	}

	s_aws3_param(int _id, const char * _str, const char * _exp, short * _s) :id(_id), str(_str), exp(_exp), type(MAV_PARAM_TYPE_INT16), s(_s), key(-1), sync(false)
	{
	}

	s_aws3_param(int _id, const char * _str, const char * _exp, int * _i) :id(_id), str(_str), exp(_exp), type(MAV_PARAM_TYPE_INT32), i(_i), key(-1), sync(false)
	{
	}

	s_aws3_param(int _id, const char * _str, const char * _exp, float * _f) :id(_id), str(_str), exp(_exp), type(MAV_PARAM_TYPE_REAL32), f(_f), key(-1), sync(false)
	{
	}
	s_aws3_param(int _id, const char * _str, const char * _exp, double * _d) :id(_id), str(_str), exp(_exp), type(MAV_PARAM_TYPE_REAL64), d(_d), key(-1), sync(false)
	{
	}

	int key;
	bool sync;

	void set(float val)
	{
		switch (type){
		case MAV_PARAM_TYPE_UINT8:
		  *uc = (unsigned char) val;
		  break;
		case MAV_PARAM_TYPE_UINT16:
		  *us = (unsigned short) val; 
		  break;
		case MAV_PARAM_TYPE_UINT32:
		  *ui = (unsigned int) val;
		  break;
		case MAV_PARAM_TYPE_INT8:
		  *c = (char)val;
		  break;
		case MAV_PARAM_TYPE_INT16:
		  *s = (short)val;
		  break;
		case MAV_PARAM_TYPE_INT32:
		  *i = (int)val;
		  break;
		case MAV_PARAM_TYPE_REAL32:
		  *f = val;
		  break;
		case MAV_PARAM_TYPE_REAL64:
		  *d = val;
		  break;
		}
		sync = true;
	};

	void get(float & val)
	{
		switch (type){
		case MAV_PARAM_TYPE_UINT8:
		  val = *uc;
		  break;
		case MAV_PARAM_TYPE_UINT16:
		  val = (float) *us;
		  break;
		case MAV_PARAM_TYPE_UINT32:		  
		  val = (float) * ui;
		  break;
		case MAV_PARAM_TYPE_INT8:
		  val = *c;
		  break;
		case MAV_PARAM_TYPE_INT16:
		  val = *s;
		  break;
		case MAV_PARAM_TYPE_INT32:
		  val = (float)*i;
		  break;
		case MAV_PARAM_TYPE_REAL32:
		  val = *f;
		  break;
		}
	}

	bool is_sync()
	{
		return sync;
	}

	bool reset()
	{
		sync = false;
	}
};

class ch_aws3_param : ch_base
{
private:
	enum { //parameter indices
		// Layout version number, always key zero.
		//
		k_param_format_version = 0,
		k_param_software_type,

		k_param_g2, // 2nd block of parameters

		k_param_sitl, // Simulation

		// Telemetry
		k_param_gcs0 = 10,
		k_param_gcs1,
		k_param_gcs2,
		k_param_gcs3,
		k_param_sysid_this_mav,
		k_param_sysid_my_gcs,

		// Hardware/Software configuration
		k_param_BoardConfig = 20, // Board configuration (PX4/Linux/etc)
		k_param_scheduler, // Scheduler (for debugging/perf_info)
		k_param_DataFlash, // DataFlash Logging
		k_param_serial_manager, // Serial ports, AP_SerialManager
		k_param_notify, // Notify Library, AP_Notify
		k_param_cli_enabled, // Old (deprecated) command line interface

		// Sensor objects
		k_param_ins = 30, // AP_InertialSensor
		k_param_compass, // Compass
		k_param_barometer, // Barometer/Depth Sensor
		k_param_battery, // AP_BattMonitor
		k_param_leak_detector, // Leak Detector
		k_param_rangefinder, // Rangefinder
		k_param_gps, // GPS
		k_param_optflow, // Optical Flow

		// Navigation libraries
		k_param_ahrs = 50, // AHRS
		k_param_NavEKF, // Extended Kalman Filter Inertial Navigationsr
		k_param_NavEKF2, // EKF2
		k_param_attitude_control, // Attitude Control
		k_param_pos_control, // Position Control
		k_param_wp_nav, // Waypoint navigation
		k_param_mission, // Mission library
		k_param_fence, // Fence Library
		k_param_terrain, // Terrain database
		k_param_rally, // Disabled
		k_param_circle_nav, // Disabled
		k_param_avoid, // Relies on proximity and fence

		// Other external hardware interfaces
		k_param_motors = 65, // Motors
		k_param_relay, // Relay
		k_param_camera, // Camera
		k_param_camera_mount, // Camera gimbal

		// RC_Channel settings
		k_param_rc_1 = 75,
		k_param_rc_2,
		k_param_rc_3,
		k_param_rc_4,
		k_param_rc_5,
		k_param_rc_6,
		k_param_rc_7,
		k_param_rc_8,
		k_param_rc_9,
		k_param_rc_10,
		k_param_rc_11,
		k_param_rc_12,
		k_param_rc_13,
		k_param_rc_14,

		// Joystick gain parameters
		k_param_gain_default,
		k_param_maxGain,
		k_param_minGain,
		k_param_numGainSettings,
		k_param_cam_tilt_step,
		k_param_lights_step,

		// Joystick button mapping parameters
		k_param_jbtn_0 = 95,
		k_param_jbtn_1,
		k_param_jbtn_2,
		k_param_jbtn_3,
		k_param_jbtn_4,
		k_param_jbtn_5,
		k_param_jbtn_6,
		k_param_jbtn_7,
		k_param_jbtn_8,
		k_param_jbtn_9,
		k_param_jbtn_10,
		k_param_jbtn_11,
		k_param_jbtn_12,
		k_param_jbtn_13,
		k_param_jbtn_14,
		k_param_jbtn_15,

		// Flight mode selection
		k_param_flight_mode1 = 120,
		k_param_flight_mode2,
		k_param_flight_mode3,
		k_param_flight_mode4,
		k_param_flight_mode5,
		k_param_flight_mode6,

		// PID Controllers
		k_param_p_pos_xy,
		k_param_p_alt_hold,
		k_param_pi_vel_xy,
		k_param_p_vel_z,
		k_param_pid_accel_z,
		k_param_pid_crosstrack_control, // Experimental
		k_param_pid_heading_control, // Experimental

		// Failsafes
		k_param_failsafe_gcs = 140,
		k_param_failsafe_leak, // leak failsafe behavior
		k_param_failsafe_pressure, // internal pressure failsafe behavior
		k_param_failsafe_pressure_max, // maximum internal pressure in pascal before failsafe is triggered
		k_param_failsafe_temperature, // internal temperature failsafe behavior
		k_param_failsafe_temperature_max, // maximum internal temperature in degrees C before failsafe is triggered
		k_param_failsafe_terrain, // terrain failsafe behavior
		k_param_fs_ekf_thresh,
		k_param_fs_ekf_action,
		k_param_fs_crash_check,
		k_param_failsafe_battery_enabled,
		k_param_fs_batt_mah,
		k_param_fs_batt_voltage,
		k_param_failsafe_throttle,
		k_param_failsafe_throttle_value,

		// Misc Sub settings
		k_param_log_bitmask = 165,
		k_param_arming_check,
		k_param_angle_max,
		k_param_rangefinder_gain,
		k_param_gps_hdop_good,
		k_param_wp_yaw_behavior,
		k_param_xtrack_angle_limit, // Angle limit for crosstrack correction in Auto modes (degrees)
		k_param_pilot_velocity_z_max,
		k_param_pilot_accel_z,
		k_param_compass_enabled,
		k_param_surface_depth,
		k_param_rc_speed, // Main output pwm frequency
		k_param_esc_calibrate, // Boot-time ESC calibration behavior
		k_param_gcs_pid_mask,
		k_param_throttle_filt,
		k_param_throttle_deadzone, // Used in auto-throttle modes
		k_param_disarm_delay,
		k_param_terrain_follow,
		k_param_rc_feel_rp,

		// Acro Mode parameters
		k_param_acro_yaw_p = 220, // Used in all modes for get_pilot_desired_yaw_rate
		k_param_acro_trainer,
		k_param_acro_expo,
		k_param_acro_rp_p,
		k_param_acro_balance_roll,
		k_param_acro_balance_pitch,

		// AUX switch options
		k_param_ch7_option, // Disabled
		k_param_ch8_option, // Disabled
		k_param_ch9_option, // Disabled
		k_param_ch10_option, // Disabled
		k_param_ch11_option, // Disabled
		k_param_ch12_option, // Disabled

		// RPM Sensor
		k_param_rpm_sensor, // Disabled

		// RC_Mapper Library
		k_param_rcmap, // Disabled

		// CH6 Tuning
		k_param_radio_tuning, // Disabled
		k_param_radio_tuning_high, // Disabled
		k_param_radio_tuning_low, // Disabled

		// Autotune parameters
		k_param_autotune_axis_bitmask, // Disabled
		k_param_autotune_aggressiveness, // Disabled
		k_param_autotune_min_d, // Disabled
	};

	short format_version;
	char  software_type;

	// Telemetry control
	//
	short sysid_this_mav;
	short sysid_my_gcs;
	char  cli_enabled;

	float throttle_filt;
	float tkoff_alt;
	float tkoff_dz;
	char  thr_bhv;

	struct Serial{
		short baud;
		char protocol;
	} serial[6];

	char  telem_delay;
	short rtl_alt;
	float rtl_cone_slope;
	short rtl_speed;

	float rangefinder_gain;

	char  failsafe_battery_enabled;   // battery failsafe enabled
	float fs_batt_voltage;            // battery voltage below which failsafe will be triggered
	float fs_batt_mah;                // battery capacity (in mah) below which failsafe will be triggered

	char  failsafe_leak;				// leak detection failsafe behavior
	char  failsafe_gcs;               // ground station failsafe behavior
	char  failsafe_pressure;
	char  failsafe_temperature;
	int	  failsafe_pressure_max;
	char  failsafe_temperature_max;
	char  failsafe_terrain;

	char  xtrack_angle_limit;

	short gps_hdop_good;              // GPS Hdop value at or below this value represent a good position

	char  compass_enabled;
	char  super_simple;
	short rtl_alt_final;
	short rtl_climb_min;
	char  wp_yaw_behavior;            // controls how the autopilot controls yaw during missions
	int   rtl_loit_time;
	short land_speed;
	short land_speed_high;

	char  rc_feel_rp;                 // controls vehicle response to user input with 0 being extremely soft and 100 begin extremely crisp
	char  land_reposition;

	// Waypoints
	//
	short pilot_velocity_z_max;        // maximum vertical velocity the pilot may request
	short pilot_accel_z;               // vertical acceleration the pilot may request

	// Throttle
	//
	char  failsafe_throttle;
	short failsafe_throttle_value;
	short throttle_deadzone;

	// Flight modes
	//
	char  flight_mode1;
	char  flight_mode2;
	char  flight_mode3;
	char  flight_mode4;
	char  flight_mode5;
	char  flight_mode6;

	char simple;

	// Misc
	//
	int   log_bitmask;
	char  esc_calibrate;
	char  radio_tuning;
	short radio_tuning_high;
	short radio_tuning_low;
	char  frame;
	char  ch7_option;
	char  ch8_option;
	char  ch9_option;
	char  ch10_option;
	char  ch11_option;
	char  ch12_option;
	char  arming_check;
	char  disarm_delay;

	char  fs_ekf_action;
	char  fs_crash_check;
	float fs_ekf_thresh;
	short gcs_pid_mask;

	char  terrain_follow;

	// RC channels
	struct RC{
		short dz;
		short max;
		short min;
		short rev;
		short trim;
	};

	struct RCA{
		short dz;
		short function;
		short max;
		short min;
		short rev;
		short trim;
	};

	RC  rc_1;
	RC  rc_2;
	RC  rc_3;
	RC  rc_4;
	RCA rc_5;
	RCA rc_6;
	RCA rc_7;
	RCA rc_8;
	RCA rc_9;
	RCA rc_10;
	RCA rc_11;
	RCA rc_12;
	RCA rc_13;
	RCA rc_14;

	short rc_speed; // speed of fast RC Channels in Hz

	float gain_default;
	float maxGain;
	float minGain;
	char numGainSettings;

	short cam_tilt_step;
	short lights_step;

	// Joystick button parameters
	typedef enum
	{
		k_none = 0,            ///< disabled
		k_shift = 1,            ///< "shift" buttons to allow more functions
		k_arm_toggle = 2,            ///< arm/disarm vehicle toggle
		k_arm = 3,            ///< arm vehicle
		k_disarm = 4,            ///< disarm vehicle
		k_mode_toggle = 5,            ///< toggle through available modes
		k_mode_1 = 6,            ///< enter mode 1
		k_mode_2 = 7,            ///< enter mode 2
		k_mode_3 = 8,            ///< enter mode 3
		k_mode_4 = 9,            ///< enter mode 4
		k_mode_5 = 10,           ///< enter mode 5
		k_mode_6 = 11,           ///< enter mode 6
		// 12-20 reserved for future mode functions
		k_mount_center = 21,           ///< move mount to center
		k_mount_tilt_up = 22,           ///< tilt mount up
		k_mount_tilt_down = 23,           ///< tilt mount down
		k_camera_trigger = 24,           ///< trigger camera shutter
		k_camera_source_toggle = 25,           ///< toggle camera source
		k_mount_pan_right = 26,           ///< pan mount right
		k_mount_pan_left = 27,           ///< pan mount left
		// 26-30 reserved for future camera functions
		k_lights1_cycle = 31,           ///< lights 1 cycle
		k_lights1_brighter = 32,           ///< lights 1 up
		k_lights1_dimmer = 33,           ///< lights 1 down
		k_lights2_cycle = 34,           ///< lights 2 cycle
		k_lights2_brighter = 35,           ///< lights 2 up
		k_lights2_dimmer = 36,           ///< lights 2 down
		// 37-40 reserved for future light functions
		k_gain_toggle = 41,           ///< toggle different gain settings
		k_gain_inc = 42,           ///< increase control gain
		k_gain_dec = 43,           ///< decrease control gain
		k_trim_roll_inc = 44,           ///< increase roll trim
		k_trim_roll_dec = 45,           ///< decrease roll trim
		k_trim_pitch_inc = 46,           ///< increase pitch trim
		k_trim_pitch_dec = 47,           ///< decrease pitch trim
		k_input_hold_toggle = 48,           ///< toggle input hold (trim to current controls)
		// 49-50 reserved for future functions
		k_relay_1_on = 51,           ///< trigger relay on
		k_relay_1_off = 52,           ///< trigger relay off
		k_relay_1_toggle = 53,           ///< trigger relay toggle
		k_relay_2_on = 54,           ///< trigger relay on
		k_relay_2_off = 55,           ///< trigger relay off
		k_relay_2_toggle = 56,           ///< trigger relay toggle
		// 57-90 reserved for future functions
		k_custom_1 = 91,           ///< custom user button 1
		k_custom_2 = 92,           ///< custom user button 2
		k_custom_3 = 93,           ///< custom user button 3
		k_custom_4 = 94,           ///< custom user button 4
		k_custom_5 = 95,           ///< custom user button 5
		k_custom_6 = 96,           ///< custom user button 6
		// 97+ reserved for future functions
		k_nr_btn_functions         ///< This must be the last enum value (only add new values _before_ this one)
	} button_function_t;
 
	char jbtn_0;
	char jbtn_1;
	char jbtn_2;
	char jbtn_3;
	char jbtn_4;
	char jbtn_5;
	char jbtn_6;
	char jbtn_7;
	char jbtn_8;
	char jbtn_9;
	char jbtn_10;
	char jbtn_11;
	char jbtn_12;
	char jbtn_13;
	char jbtn_14;
	char jbtn_15;
	char jbtn_s_0;
	char jbtn_s_1;
	char jbtn_s_2;
	char jbtn_s_3;
	char jbtn_s_4;
	char jbtn_s_5;
	char jbtn_s_6;
	char jbtn_s_7;
	char jbtn_s_8;
	char jbtn_s_9;
	char jbtn_s_10;
	char jbtn_s_11;
	char jbtn_s_12;
	char jbtn_s_13;
	char jbtn_s_14;
	char jbtn_s_15;

	// Acro parameters
	float acro_rp_p;
	float acro_yaw_p;
	float acro_balance_roll;
	float acro_balance_pitch;
	char  acro_trainer;
	float acro_expo;

	// PI/D controllers
	float xy_filt_hz;
	float xy_i;
	float xy_imax;
	float xy_p;
	float z_p;
	float accel_z_d;
	float accel_z_filt;
	float accel_z_i;
	float accel_z_imax;
	float accel_z_p;
	float pos_xy_p;
	float pos_z_p;

	short angle_max; // common parameter for all types of vehicles

#if TRANSECT_ENABLED == ENABLED
	//AC_PID pid_crosstrack_control;
	//AC_PID pid_heading_control;
#endif


	// Autotune
#if AUTOTUNE_ENABLED == ENABLED
	//char                 autotune_axis_bitmask;
	//float                autotune_aggressiveness;
	//float                autotune_min_d;
#endif

	float				surface_depth;

	struct s_camera{
		short center;
		short duration;
		short feedback_pin;
		short feedback_pol;
		short max_roll;
		short min_interval;
		short relay_on;
		short servo_off;
		short servo_on;
		float trigg_dist;
		short trigg_type;
	} cam;

	struct s_relay{
		short def;
		short pin;
		short pin2;
		short pin3;
		short pin4;
	} relay;

	struct s_gripper
	{
		char enable;
		char type;
		short grab;
		short release;
		short neutral;
		short seconds;
		short regrab;
		short uavcan_id;
	} gripper;

	short lgr_servo_rtract;
	short lgr_servo_deploy;

	struct s_compass{
		char autodec;
		float cal_fit;
		float dec;
		int dev_id;
		int dev_id2;
		int dev_id3;
		float dia2_x;
		float dia2_y;
		float dia2_z;
		float dia3_x;
		float dia3_y;
		float dia3_z;
		float dia_x;
		float dia_y;
		float dia_z;
		char extern2;
		char extern3;
		char external;
		char learn;
		float mot2_x;
		float mot2_y;
		float mot2_z;
		float mot3_x;
		float mot3_y;
		float mot3_z;
		float mot_x;
		float mot_y;
		float mot_z;
		char motct;
		float odi2_x;
		float odi2_y;
		float odi2_z;
		float odi3_x;
		float odi3_y;
		float odi3_z;
		float odi_x;
		float odi_y;
		float odi_z;
		float ofs2_x;
		float ofs2_y;
		float ofs2_z;
		float ofs3_x;
		float ofs3_y;
		float ofs3_z;
		float ofs_x;
		float ofs_y;
		float ofs_z;
		char orient;
		char orient2;
		char orient3;
		char primary;
		char use;
		char use2;
		char use3;

	} compass;

	struct s_ins{
		float acc3offs_x;
		float acc3offs_y;
		float acc3offs_z;
		float acc3scal_x;
		float acc3scal_y;
		float acc3scal_z;
		float acc2offs_x;
		float acc2offs_y;
		float acc2offs_z;
		float acc2scal_x;
		float acc2scal_y;
		float acc2scal_z;
		float accoffs_x;
		float accoffs_y;
		float accoffs_z;
		float accscal_x;
		float accscal_y;
		float accscal_z;
		short accel_filter;
		char acc_bodyfix;
		int acc2_id;
		int acc3_id;
		int acc_id;
		char fast_sample;
		float gyr3offs_x;
		float gyr3offs_y;
		float gyr3offs_z;
		float gyr2offs_x;
		float gyr2offs_y;
		float gyr2offs_z;
		float gyroffs_x;
		float gyroffs_y;
		float gyroffs_z;
		short gyro_filter;
		char gyro_cal;
		int gyr3_id;
		int gyr2_id;
		int gyr_id;
		float pos1_x;
		float pos1_y;
		float pos1_z;
		float pos2_x;
		float pos2_y;
		float pos2_z;
		float pos3_x;
		float pos3_y;
		float pos3_z;
		int product_id;
		float still_thresh;
		char trim_option;
		char use;
		char use2;
		char use3;
	} ins;

	short circle_radius;
	char circle_rate;

	struct s_wpnav{
		float accel;
		float accel_z;
		float loit_jerk;
		float loit_maxa;
		float loit_mina;
		float loit_speed;
		float radius;
		char rfnd_use;
		float speed;
		float speed_dn;
		float speed_up;
	} wpnav;

	struct s_atc{
		float accel_p_max;
		float accel_r_max;
		float accel_y_max;
		char angle_boost;
		float ang_lim_tc;
		float ang_pit_p;
		float ang_rll_p;
		float ang_yaw_p;
		char rate_ff_enab;
		float rat_pit_d;
		float rat_pit_filt;
		float rat_pit_i;
		float rat_pit_imax;
		float rat_pit_p;
		float rat_rll_d;
		float rat_rll_filt;
		float rat_rll_i;
		float rat_rll_imax;
		float rat_rll_p;
		float rat_yaw_d;
		float rat_yaw_filt;
		float rat_yaw_i;
		float rat_yaw_imax;
		float rat_yaw_p;
		float slew_yaw;
		float thr_mix_max;
		float thr_mix_min;
	} atc;

	struct s_sr{
		char extra1;
		char extra2;
		char extra3;
		char ext_stat;
		char params;
		char position;
		char raw_ctrl;
		char raw_sens;
		char rc_chan;
		char adsb;
	};

	s_sr sr0;
	s_sr sr1;
	s_sr sr2;
	s_sr sr3;

	float psc_acc_xy_filt;

	struct s_ahrs
	{
		float comp_beta;
		char ekf_type;
		float gps_gain;
		short gps_minsats;
		char gps_use;
		short orientation;
		float rp_p;
		float trim_x;
		float trim_y;
		float trim_z;
		short wind_max;
		float yaw_p;
	} ahrs;

#if MOUNT == ENABLED
	struct s_mnt
	{
		char deflt_mode;
		short angmax_pan;
		short angmax_rol;
		short angmax_til;
		short angmin_pan;
		short angmin_rol;
		short angmin_til;
		char jstick_spd;
		float lead_pitch;
		float lead_rll;
		float neutral_x;
		float neutral_y;
		float neutral_z;
		char rc_in_pan;
		char rc_in_roll;
		char rc_in_tilt;
		float retract_x;
		float retract_y;
		float retract_z;
		char stab_pan;
		char stab_roll;
		char stab_tilt;
		char type;
	} mnt;
#endif

	struct s_log
	{
		char backend_type;
		char disarmed;
		short file_bufsize;
		char file_dsrmrot;
		char replay;
	} log;

	struct s_batt
	{
		float amp_offset;
		float amp_pervol;
		short capacity;
		char curr_pin;
		char monitor;
		float volt_mult;
		char volt_pin;
	};

	s_batt batt;
	s_batt batt2;

	struct s_brd
	{
		char can_enable;
		char imu_targetemp;
		char pwm_count;
		char safetyenable;
		char safety_mask;
		char sbus_out;
		char ser1_rtscts;
		char ser2_rtscts;
		char serial_num;
		char type;
	} brd;

	struct s_gnd{
		float abs_press;
		float alt_offset;
		float base_press;
		char base_reset;
		char primary;
		float spec_grav;
		float temp;
	} gnd;

	struct s_gps{
		char auto_config;
		char auto_switch;
		char gnss_mode;
		char gnss_mode2;
		short hdop_good;
		short inject_to;
		short min_dgps;
		short min_elevation;
		short navfilter;
		float pos1_x;
		float pos1_y;
		float pos1_z;
		float pos2_x;
		float pos2_y;
		float pos2_z;
		float rate_ms;
		float rate_ms2;
		char raw_data;
		char save_cfg;
		char sbas_mode;
		short sbp_logmask;
		char type;
		char type2;
	} gps;

	struct s_leak
	{
		char logic;
		char pin;
	};

	s_leak leak1;
	s_leak leak2;
	s_leak leak3;

	struct s_sched
	{
		char debug;
		short loop_rate;
	} sched;

	char avoid_enable;

#if AC_FENCE == ENABLED
	struct s_fence
	{
		char action;
		float alt_max;
		float depth_max;
		char enable;
		float margin;
		float radius;
		char total;
		char type;
	} fence;
#endif

	struct s_mot
	{
		float bat_curr_max;
		float bat_curr_tc;
		float bat_volt_max;
		float bat_volt_min;
		float fv_cplng_k;
		char hover_learn;
		short pwm_max;
		short pwm_min;
		char pwm_type;
		char safe_disarm;
		float spin_arm;
		float spin_max;
		float spin_min;
		float thst_expo;
		float thst_hover;
		short yaw_headroom;
		char direction[8];
	} mot;

#if RCMAP_ENABLED == ENABLED
	struct s_rcmap{
		char roll;
		char pitch;
		char throttle;
		char yaw;
		char forward;
		char lateral;
	} rcmap;
#endif

	struct s_ekf
	{
		char enable;
		float velne_noise;
		float veld_noise;
		float posne_noise;
		float alt_noise;
		float mag_noise;
		float eas_noise;
		float wind_pnoise;
		float wind_pscale;
		float gyro_pnoise;
		float acc_pnoise;
		float gbias_pnoise;
		float abias_pnoise;
		float mage_pnoise;
		float magb_pnoise;
		float vel_delay;
		float pos_delay;
		char gps_type;
		short vel_gate;
		short pos_gate;
		short hgt_gate;
		short mag_gate;
		short eas_gate;
		char mag_cal;
		short glitch_accel;
		short glitch_rad;
		short gnd_gradient;
		float flow_noise;
		short flow_gate;
		short flow_delay;
		short rng_gate;
		float max_flow;
		char fallback;
		char alt_source;
		char gps_check;
	} ekf;

	struct s_ek2
	{
		float abias_p_nse;
		float acc_p_nse;
		float alt_m_nse;
		char alt_source;
		short bcn_delay;
		short bcn_i_gte;
		float bcn_m_nse;
		char enable;
		short flow_delay;
		short flow_i_gate;
		float flow_m_nse;
		float gbias_p_nse;
		short glitch_rad;
		short gps_check;
		short gps_delay;
		short gps_type;
		float eas_m_nse;
		short eas_i_gate;

		float gscl_p_nse;
		float gyro_p_nse;
		short hgt_delay;
		short hgt_i_gate;
		char imu_mask;
		char log_mask;
		float magb_p_nse;
		float mage_p_nse;
		char mag_cal;
		short mag_i_gate;
		float mag_m_nse;
		float max_flow;
		float noaid_m_nse;
		float posne_m_nse;
		short pos_i_gate;
		short rng_i_gate;
		float rng_m_nse;
		char rng_use_hgt;
		char tau_output;
		float terr_grad;
		float veld_m_nse;
		float velne_m_nse;
		short vel_i_gate;
		float wind_pscale;
		float wind_p_nse;
		short yaw_i_gate;
		float yaw_m_nse;
		short check_scale;
	} ek2;

	struct s_mis{
		char restart;
		short total;
	} mis;

	struct s_rssi{
		char type;
		char ana_pin;
		float pin_low;
		float pin_high;
		char channel;
		short chan_low;
		short chan_high;
	} rssi;

	struct s_rngfnd{
		char type;
		char pin;
		float scaling;
		float offset;
		char function;
		short min_cm;
		short max_cm;
		char stop_pin;
		short settle;
		char rmetric;
		short pwrrng;
		char gndclear;
		char addr;
		short pos_x;
		short pos_y;
		short pos_z;
	} rngfnd[2];

	struct s_flow{
		char enable;
		short fxscaler;
		short fyscaler;
		short orient_yaw;
		short pos_x;
		short pos_y;
		short pos_z;
		short bus_id;
	} flow;

	struct s_rpm{
		char type;
		float scaling;
		short max_rpm;
		short min_rpm;
		float min_qual;
		char type2;
		float scaling2;
	} rpm;

	char autotune_axes;
	float autotune_aggr;
	float autotune_min_d;

	struct s_ntf{
		char buzz_enable;
		char led_brright;
		char led_override;
	} ntf;

	char throw_motor_start;

#define SIZE_HTBL 2003
	vector<int> m_htbl;		// hash table
	vector<s_aws3_param> m_ptbl;
	s_aws3_param m_noparam;
	void create_param(f_aws3_com * paws3c, int id, const char * str, const char * exp, char * c);
	void create_param(f_aws3_com * paws3c, int id, const char * str, const char * exp, short * s);
	void create_param(f_aws3_com * paws3c, int id, const char * str, const char * exp, int * i);
	void create_param(f_aws3_com * paws3c, int id, const char * str, const char * exp, float * f);

public:
	ch_aws3_param(const char * name) :ch_base(name)
	{
	}

	virtual ~ch_aws3_param()
	{
	}

	void register_param(f_aws3_com * paws3c);

	int hash(const char * str){
		int key = 0;
		for (int i = 0; i < 16; i++){
			if (str[i] != '\0')
			{
				key += (int)str[i];
			}
			else{ break; }
		}
		key = key % SIZE_HTBL;
		return key;
	}

	int seek_param(const char * str){
		int key = hash(str);
		int ipar;
		while ((ipar = m_htbl[key]) >= 0){
			const char * pstr = m_ptbl[ipar].str;
			bool eq = true;
			for (int i = 0; i < 16 && pstr[i] != '\0' && str[i] != '\0'; i++)
			{
				if (pstr[i] != str[i]){
					eq = false;
					break;
				}
			}
			if (eq)
				break;
			else{
				key++;
				if (key == SIZE_HTBL)
					key = 0;
			}
			ipar = -1;
		}
		return ipar;
	}

	s_aws3_param & get_param(int iparam);
	bool check_sync();
	bool set_value(mavlink_param_value_t & pv);

	int get_num_params()
	{
		return (int)m_ptbl.size();
	}
};

class ch_aws3_state: ch_base
{
public:
	// Auto Pilot Modes enumeration(custom mode)
	enum control_mode_t {
		STABILIZE = 0,  // manual angle with manual depth/throttle
		ACRO = 1,  // manual body-frame angular rate with manual depth/throttle
		ALT_HOLD = 2,  // manual angle with automatic depth/throttle
		AUTO = 3,  // not implemented in sub // fully automatic waypoint control using mission commands
		GUIDED = 4,  // not implemented in sub // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
		VELHOLD = 5,  // automatic x/y velocity control and automatic depth/throttle
		RTL = 6,  // not implemented in sub // automatic return to launching point
		CIRCLE = 7,  // not implemented in sub // automatic circular flight with automatic throttle
		SURFACE = 9,  // automatically return to surface, pilot maintains horizontal control
		OF_LOITER = 10,  // deprecated
		TRANSECT = 13,  // automatic x/y velocity, automatic heading/crosstrack error compensation, automatic depth/throttle
		AUTOTUNE = 15,  // not implemented in sub // automatically tune the vehicle's roll and pitch gains
		POSHOLD = 16,  // automatic position hold with manual override, with automatic throttle
		MANUAL = 19   // Pass-through input with no stabilization
	};

private:
	long long thb; // heart beat time
	uint8_t base_mode;
	uint32_t custom_mode;
	uint8_t system_status;

public:
	ch_aws3_state(const char * name) : ch_base(name)
	{
	}

	virtual ~ch_aws3_state()
	{
	}

	void set_alive(const long long _thb)
	{
		thb = _thb;
	}

	bool is_alive(const long long _thb_last)
	{
		return thb > _thb_last;
	}

	// handler of base_mode 
	void set_base_mode(const uint8_t _base_mode)
	{
		base_mode = _base_mode;
	}

	bool is_safety_armed()
	{
		return (MAV_MODE_FLAG_SAFETY_ARMED & base_mode) != 0;
	}

	bool is_manual_input_enabled()
	{
		return (MAV_MODE_FLAG_MANUAL_INPUT_ENABLED & base_mode) != 0;
	}

	bool is_hil_enabled()
	{
		return (MAV_MODE_FLAG_HIL_ENABLED & base_mode) != 0;
	}

	bool is_stabilize_enabled()
	{
		return (MAV_MODE_FLAG_STABILIZE_ENABLED & base_mode) != 0;
	}

	bool is_guided_enabled()
	{ 
		return (MAV_MODE_FLAG_GUIDED_ENABLED & base_mode) != 0;
	}

	bool is_auto_enabled()
	{
		return (MAV_MODE_FLAG_AUTO_ENABLED & base_mode) != 0;
	}

	bool is_test_enabled()
	{
		return (MAV_MODE_FLAG_TEST_ENABLED & base_mode) != 0;
	}

	bool is_custom_mode_enabled()
	{
		return (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED & base_mode) != 0;
	}
	// handler of custom_mode
	void set_custom_mode(const uint8_t _custom_mode)
	{
		custom_mode = _custom_mode;
	}

	// handler of mav_state
	void set_system_status(const uint8_t _system_status)
	{
		// MAV_STATE_STANDBY means not armed/ MAV_STATE_ACTIVE means armed
		system_status = _system_status;
	}


};

class ch_aws3_cmd: ch_base
{
private:
	bool m_jbtns[16];
	short m_jx, m_jy, m_jz, m_jr;

public:
	ch_aws3_cmd(const char * name) : ch_base(name)
	{
	}

	virtual ~ch_aws3_cmd()
	{
	}


};

#endif
