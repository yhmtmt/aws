#ifndef _F_AWS3_COM_H_
#define _F_AWS3_COM_H_
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_aws3_com.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws3_com.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws3_com.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "../channel/ch_base.h"
#include "f_base.h"

#include <mavlink.h>

class f_aws3_com : f_base
{
public:
	struct s_pelem{
		int id;
		const char * str;
		const char * exp;
		union {
			char * c;
			short * s;
			int * i;
			float * f;
		};
		int type;

		s_pelem() : id(-1), str(NULL), exp(NULL), c(NULL)
		{};

		s_pelem(int _id, const char * _str, const char * _exp, char * _c) :id(_id), str(_str), exp(_exp), type(0), c(_c), key(-1), sync(false)
		{
		}

		s_pelem(int _id, const char * _str, const char * _exp, short * _s) :id(_id), str(_str), exp(_exp), type(1), s(_s), key(-1), sync(false)
		{
		}

		s_pelem(int _id, const char * _str, const char * _exp, int * _i) :id(_id), str(_str), exp(_exp), type(2), i(_i), key(-1), sync(false)
		{
		}

		s_pelem(int _id, const char * _str, const char * _exp, float * _f) :id(_id), str(_str), exp(_exp), type(3), f(_f), key(-1), sync(false)
		{
		}

		int key;
		bool sync;

		void set(float val)
		{
			switch (type){
			case 0:
				*c = (char)val;
				break;
			case 1:
				*s = (short)val;
				break;
			case 2:
				*i = (int)val;
				break;
			case 3:
				*f = val;
			}
			sync = true;
		};

		void get(float & val)
		{
			switch (type){
			case 0:
				val = *c;
				break;
			case 1:
				val = *s;
				break;
			case 2:
				val = (float) *i;
				break;
			case 3:
				val = *f;
			}
			sync = true;

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

	void create_param(int _id, const char * _str, const char * _exp, char * _c)
	{
		register_fpar(_str, _c, _exp);
		m_ptbl.push_back(s_pelem(_id, _str, _exp, _c));
	}

	void create_param(int _id, const char * _str, const char * _exp, short * _s)
	{
		register_fpar(_str, _s, _exp);
		m_ptbl.push_back(s_pelem(_id, _str, _exp, _s));
	}

	void create_param(int _id, const char * _str, const char * _exp, int * _i)
	{
		register_fpar(_str, _i, _exp);
		m_ptbl.push_back(s_pelem(_id, _str, _exp, _i));
	}

	void create_param(int _id, const char * _str, const char * _exp, float * _f)
	{
		register_fpar(_str, _f, _exp);
		m_ptbl.push_back(s_pelem(_id, _str, _exp, _f));
	}


protected:
	unsigned char m_sys_id;

	enum e_state{
		INIT = 0, LOAD_PARAM, ACTIVE
	} m_state;

	unsigned short m_port;
	SOCKET m_sock;
	sockaddr_in m_sock_addr_rcv, m_sock_addr_snd;
	socklen_t m_sz;

	uint8_t m_buf[2048];

	bool m_brst;
	bool m_bcon;

	// from aws3
	mavlink_heartbeat_t m_heartbeat;
	mavlink_raw_imu_t m_raw_imu;
	mavlink_scaled_imu2_t m_scaled_imu2;
	mavlink_scaled_pressure_t m_scaled_pressure;
	mavlink_scaled_pressure2_t m_scaled_pressure2;
	mavlink_sys_status_t m_sys_status;
	mavlink_power_status_t m_power_status;
	mavlink_mission_current_t m_mission_current;
	mavlink_system_time_t m_system_time;
	mavlink_nav_controller_output_t m_nav_controller_output;
	mavlink_global_position_int_t m_global_position_int;
	mavlink_servo_output_raw_t m_servo_output_raw;
	mavlink_rc_channels_raw_t m_rc_channels_raw;
	mavlink_attitude_t m_attitude;
	//	mavlink_rally_fetch_point_t m_rally_fetch_point;
	mavlink_vfr_hud_t m_vfr_hud;
	mavlink_hwstatus_t m_hwstatus;
	mavlink_mount_status_t m_mount_status;
	mavlink_ekf_status_report_t m_ekf_status_report;
	mavlink_vibration_t m_vibration;
	mavlink_sensor_offsets_t m_sensor_offsets;
	mavlink_rangefinder_t m_rangefinder;
	mavlink_rpm_t m_rpm;
	mavlink_camera_feedback_t m_camera_feedback;
	mavlink_limits_status_t m_limits_status;
	mavlink_simstate_t m_simstate;
	mavlink_meminfo_t m_meminfo;
	mavlink_battery2_t m_battery2;
	mavlink_gimbal_report_t m_gimbal_report;
	mavlink_pid_tuning_t m_pid_tuning;
	mavlink_mag_cal_progress_t m_mag_cal_progress;
	mavlink_mag_cal_report_t m_mag_cal_report;
	mavlink_ahrs_t m_ahrs;
	mavlink_ahrs2_t m_ahrs2;
	mavlink_ahrs3_t m_ahrs3;

	mavlink_statustext_t m_statustext;
	mavlink_param_value_t m_param_value;

	// to aws3
	bool m_jbtns[16];
	short m_jx, m_jy, m_jz, m_jr;

	//Params (almost from ArduSub's Parameters.h)
#define SIZE_HTBL 2003
	vector<int> m_htbl;		// hash table
	vector<s_pelem> m_ptbl;	// parameter table

	int hash(const char * str){
		int key = 0;
		for (int i = 0; i < 16; i++){
			if (str[i] != '\0')
			{
				key += (int)str[i];
			}
			else{ break; }
		}
		key =  key % SIZE_HTBL;
		return key;
	}

	int seek_param(const char * str){
	  cout.write(str, 16);
	  cout << endl; 
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

	short        format_version;
	char         software_type;

	// Telemetry control
	//
	short        sysid_this_mav;
	short        sysid_my_gcs;
#if CLI_ENABLED == ENABLED
	char         cli_enabled;
#endif

	float        throttle_filt;

	float        rangefinder_gain;

	char         failsafe_battery_enabled;   // battery failsafe enabled
	float        fs_batt_voltage;            // battery voltage below which failsafe will be triggered
	float        fs_batt_mah;                // battery capacity (in mah) below which failsafe will be triggered

	char			failsafe_leak;				// leak detection failsafe behavior
	char         failsafe_gcs;               // ground station failsafe behavior
	char			failsafe_pressure;
	char			failsafe_temperature;
	int		failsafe_pressure_max;
	char			failsafe_temperature_max;
	char			failsafe_terrain;

	char			xtrack_angle_limit;

	short        gps_hdop_good;              // GPS Hdop value at or below this value represent a good position

	char         compass_enabled;

	char         wp_yaw_behavior;            // controls how the autopilot controls yaw during missions
	char         rc_feel_rp;                 // controls vehicle response to user input with 0 being extremely soft and 100 begin extremely crisp

	// Waypoints
	//
	short        pilot_velocity_z_max;        // maximum vertical velocity the pilot may request
	short        pilot_accel_z;               // vertical acceleration the pilot may request

	// Throttle
	//
	char         failsafe_throttle;
	short        failsafe_throttle_value;
	short        throttle_deadzone;

	// Flight modes
	//
	char         flight_mode1;
	char         flight_mode2;
	char         flight_mode3;
	char         flight_mode4;
	char         flight_mode5;
	char         flight_mode6;

	// Misc
	//
	int        log_bitmask;
	char         esc_calibrate;
#if CH6_TUNE_ENABLED == ENABLED
	char         radio_tuning;
	short        radio_tuning_high;
	short        radio_tuning_low;
#endif
	char         ch7_option;
	char         ch8_option;
	char         ch9_option;
	char         ch10_option;
	char         ch11_option;
	char         ch12_option;
	char         arming_check;
	char         disarm_delay;

	char         fs_ekf_action;
	char         fs_crash_check;
	float        fs_ekf_thresh;
	short        gcs_pid_mask;

	char         terrain_follow;

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

	RC              rc_1;
	RC              rc_2;
	RC              rc_3;
	RC              rc_4;
	RCA          rc_5;
	RCA          rc_6;
	RCA          rc_7;
	RCA          rc_8;
	RCA          rc_9;
	RCA          rc_10;
	RCA          rc_11;
	RCA          rc_12;
	RCA          rc_13;
	RCA          rc_14;

	short                rc_speed; // speed of fast RC Channels in Hz

	float				gain_default;
	float				maxGain;
	float				minGain;
	char					numGainSettings;

	short					cam_tilt_step;
	short					lights_step;

	// Joystick button parameters
	char 				jbtn_0;
	char 				jbtn_1;
	char 				jbtn_2;
	char 				jbtn_3;
	char 				jbtn_4;
	char 				jbtn_5;
	char 				jbtn_6;
	char 				jbtn_7;
	char 				jbtn_8;
	char 				jbtn_9;
	char 				jbtn_10;
	char 				jbtn_11;
	char 				jbtn_12;
	char 				jbtn_13;
	char 				jbtn_14;
	char 				jbtn_15;
	char 				jbtn_s_0;
	char 				jbtn_s_1;
	char 				jbtn_s_2;
	char 				jbtn_s_3;
	char 				jbtn_s_4;
	char 				jbtn_s_5;
	char 				jbtn_s_6;
	char 				jbtn_s_7;
	char 				jbtn_s_8;
	char 				jbtn_s_9;
	char 				jbtn_s_10;
	char 				jbtn_s_11;
	char 				jbtn_s_12;
	char 				jbtn_s_13;
	char 				jbtn_s_14;
	char 				jbtn_s_15;

	// Acro parameters
	float                acro_rp_p;
	float                acro_yaw_p;
	float                acro_balance_roll;
	float                acro_balance_pitch;
	char                 acro_trainer;
	float                acro_expo;

	// PI/D controllers
	float			xy_filt_hz;
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
		float trim_z;
		short wind_max;
		float yaw_p;
	} ahrs;

#if MOUNT == ENABLED
	struct s_mnt
	{
		short angmax_pan;
		short angmax_rol;
		short angmax_til;
		short angmin_pan;
		short angmin_rol;
		short angmin_til;
		char deflt_mode;
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
		int bitmask;
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
		char direction[6];
	} mot;

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

	struct s_ntf{
		char buzz_enable;
		char led_brright;
		char led_override;
	} ntf;

	int num_retry_load_param;
	int max_retry_load_param;
	bool load_parameters();
	void handle_param_value();
	void handle_statustext();
public:
	f_aws3_com(const char * name);
	virtual ~f_aws3_com();
	
	virtual bool init_run();
	virtual void destroy_run();

	virtual bool proc();
};

#endif
