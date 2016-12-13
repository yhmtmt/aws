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

class f_aws3_com: f_base
{

protected:

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

	// to aws3
	bool m_jbtns[16];
	short m_jx, m_jy, m_jz, m_jr;


//Params (almost from ArduSub's Parameters.h)

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
		k_param_NavEKF, // Extended Kalman Filter Inertial Navigation
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
public:
	f_aws3_com(const char * name);
	virtual ~f_aws3_com();
	
	virtual bool init_run();

	virtual void destroy_run();

	virtual bool proc();
};

#endif