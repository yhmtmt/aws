#ifndef _F_ORB_SLAM_H_
#define _F_ORB_SLAM_H_
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_orb_slam.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_orb_slam.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_orb_slam.h.  If not, see <http://www.gnu.org/licenses/>. 
//
// This program is based on ORB SLAM2. see <https://github.com/raulmur/ORB_SLAM2>


#include "../util/aws_sock.h"
#include "../util/aws_vlib.h"

#include "../channel/ch_image.h"
#include "../channel/ch_orb_slam.h"

#include "f_base.h"

#include "../orb_slam/ORBextractor.h"
#include "../orb_slam/Initializer.h"
#include "../orb_slam/Optimizer.h"

#define DEBUG_ORB_SLAM

namespace ORB_SLAM2{

	class f_tracker : public f_base
	{
	protected:
		ch_sys * m_sys;
		ch_image_ref * m_cam;
		ch_keyframeDB * m_kfdb;
		ch_keyframe * m_kf_mapper;
		ch_map * m_map;
		ch_trj * m_trj;
		ch_frm * m_frm;

		AWSCamPar m_cp;
		Mat m_Kf, m_Df; // m_cp is double precision, here the single precision camera parameters are presented
		Mat m_map1, m_map2;

		char m_fcp[1024];
		char m_fvoc[1024];
		ORBVocabulary * m_pvoc;

		ORBextractor * m_pORBEx, *m_pORBExIni;
		char m_fmask[1024];
		Mat m_mask;
		Rect m_roi;
		int m_num_features;
		float m_scale_factor;
		int m_num_levels;
		int m_th_fast_ini, m_th_fast_min;
		int m_max_frms, m_min_frms;
		bool m_brgb;

		bool m_undist;

		bool load_frm();

		void init_slam();
		Initializer * m_pinit;
		vector<int> m_ini_last_matches;
		vector<int> m_ini_matches;
		vector<Point2f> m_prev_matched_p2d;
		vector<Point3f> m_ini_p3d;

		void create_map();
		void reset();
		void update_last_frm_mps(); // it should be implemented in Frame
		bool track_rkf();
		bool track_mm();
		void update_last_frm();
		bool track_local_map();
		void update_local_map();
		void update_local_kfs();
		void update_local_pts();

		void search_local_points();
		bool reloc();

		bool need_new_kf();
		void create_new_kf();

		// internal parameters
		Mat m_img;
		long long m_timg, m_ifrm;
		Mat m_vel;
		Mat m_Tlr; // relative pose of last frame to reference key frame
		int m_num_matches_inliers;
		KeyFrame * m_pref_kf, * m_plast_kf;
		unsigned int m_last_kf_id;
		long unsigned int m_ifrm_last_reloc;
		bool m_bvo;
		list<MapPoint*> m_tmp_mps;

		Frame m_ini_frm, m_cur_frm, m_last_frm;

		// local map
		vector<KeyFrame*> m_local_kfs;
		vector<MapPoint*> m_local_mps;

		e_tracking_state m_state, m_last_state;

	public:
		f_tracker(const char * name);
		virtual ~f_tracker();
		virtual bool init_run();
		virtual void destroy_run();
		virtual bool proc();
	};

	class f_local_mapper : public f_base
	{
	protected:
		ch_sys * m_sys;
		ch_keyframe * m_kf_tracker, *m_kf_loop_closer;
		ch_map * m_map;

		list<MapPoint*> m_recent_mps;

		KeyFrame * m_cur_kf;
		void proc_new_kf();
		void cull_mps();
		void cull_kfs();
		void create_new_mps();
		void search_neighbours();

		Mat compute_F12(KeyFrame * & pKF1, KeyFrame * & pKF2);
		Mat convSkewSymmetricMatrix(const Mat & v);
	public:
		f_local_mapper(const char * name);
		virtual ~f_local_mapper();
		virtual bool init_run();
		virtual void destroy_run();
		virtual bool proc();
	};

	class f_loop_closer : public f_base
	{
	protected:
		typedef pair<set<KeyFrame*>, int> ConsistentGroup;

		ch_sys * m_sys;
		ch_keyframe * m_kf_mapper;
		ch_map * m_map;
		ch_keyframeDB * m_kfdb;

		long unsigned int m_last_loop_kf_id;
		float m_th_covisibility_consistency;
		thread * m_thread_ba;

		KeyFrame * m_cur_kf, * m_matched_kf;
		g2o::Sim3 m_Scw_g2o;
		Mat m_Scw;
		bool m_fix_scale;

		vector<ConsistentGroup> m_consistent_groups;
		vector<KeyFrame*> m_enough_consistent_candidates;
		vector<KeyFrame*> m_cur_connected_kfs;
		vector<MapPoint*> m_cur_matched_mps;
		vector<MapPoint*> m_loop_mps;

		bool detect_loop();
		bool compute_sim3();
		void correct_loop();
		void search_and_fuse(const KeyFrameAndPose & corrected_pose_map);

		void run_gba(unsigned long loop_kf_id);
	public:
		f_loop_closer(const char * name);
		~f_loop_closer();
		virtual bool init_run();
		virtual void destroy_run();
		virtual bool proc();
	};

	class f_viewer : public f_glfw_window
	{
	protected:
		ch_image_ref * m_cam;
		ch_sys * m_sys;
		ch_map * m_map;
		ch_trj * m_trj;
		ch_frm * m_frm;

		enum e_vmode{
			FRAME, MAP, MAP_AND_FRAME, UNDEF
		} m_vmode;
		static const char * m_str_vmode[UNDEF];

		int m_num_tracked;
		int m_num_tracked_vo;

		float m_sz_kf;
		float m_lw_kf;
		float m_lw_g;
		float m_sz_pt;
		float m_sz_cam;
		float m_lw_cam;

		GLdouble m_fovy, m_ar, m_zmin, m_zmax;
		GLdouble m_eyex, m_eyey, m_eyez;

		void draw_overlay_info(Mat & imWithInfo);
		void draw_mps();
		void draw_kfs();
		bool m_draw_kf, m_draw_g;
		void draw_cam(GLdouble * Twc);
		void load_cam_pose(GLdouble * Twc);
	public:
		f_viewer(const char * name);
		virtual ~f_viewer();

		virtual bool init_run();
		virtual void destroy_run();
		virtual bool proc();
	};
}
#endif
