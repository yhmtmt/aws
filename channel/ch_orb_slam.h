#ifndef _CH_ORB_SLAM_H_
#define _CH_ORB_SLAM_H_
// Copyright(c) 2016 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// ch_orb_slam.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_orb_slam.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_orb_slam.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "ch_base.h"
#include "../orb_slam/Map.h"
#include "../orb_slam/KeyFrameDatabase.h"

namespace ORB_SLAM2
{
	enum e_tracking_state{
		SYSTEM_NOT_READY = -1,
		NO_IMAGES_YET = 0,
		NOT_INITIALIZED = 1,
		OK = 2,
		LOST = 3
	};

	class ch_sys : public ch_base
	{
	protected:
		// tracker related flag
		bool m_rst_tracker;

		// mapper reltated flags
		bool m_rst_mapper;
		bool m_req_stop_mapper;
		bool m_stop_mapper;
		bool m_not_stop_mapper;
		bool m_accept_kf_mapper;
		bool m_abort_ba_mapper;
		bool m_finished_mapper;

		// loop closer and global BA reltated flags
		bool m_rst_loop_closer;
		bool m_running_gba;
		bool m_stop_gba;
		bool m_finish_gba;
	public:
		ch_sys(const char * name) : ch_base(name), m_rst_tracker(false),
			m_rst_mapper(false), m_rst_loop_closer(false), m_req_stop_mapper(false),
			m_stop_mapper(false), m_not_stop_mapper(false), m_accept_kf_mapper(false),
			m_abort_ba_mapper(false), m_finished_mapper(false),
			m_running_gba(false), m_stop_gba(true), m_finish_gba(true)
		{
		}

		void set_rst(){
			lock();
			m_rst_tracker = true;
			m_rst_mapper = true;
			m_rst_loop_closer = true;
			unlock();
		}

		bool is_rst_tracker()
		{
			lock();
			bool v = m_rst_tracker;
			unlock();
			return v;
		}

		void rst_done_tracker()
		{
			lock();
			m_rst_tracker = false;
			unlock();
		}

		bool is_rst_mapper()
		{
			lock();
			bool v = m_rst_mapper;
			unlock();
			return v;
		}

		void rst_done_mapper()
		{
			lock();
			m_rst_mapper = false;
			unlock();
		}

		bool is_finished_mapper()
		{
			lock();
			bool v = m_finished_mapper;
			unlock();
			return v;
		}

		void set_finished_mapper(const bool v)
		{
			lock();
			m_finished_mapper = v;
			unlock();
		}

		bool is_stopped_mapper()
		{
			lock();
			bool v = m_stop_mapper;
			unlock();
			return v;
		}

		bool stop_mapper()
		{

			lock();
			if (m_req_stop_mapper && !m_not_stop_mapper){
				m_stop_mapper = true;
				unlock();
				return true;
			}
			unlock();
			return false;
		}

		void release_mapper()
		{
			lock();
			m_stop_mapper = false;
			m_req_stop_mapper = false;
			unlock();
		}

		void set_stopped_mapper(bool v)
		{
			lock();
			m_stop_mapper = v;
			unlock();
		}

		void req_stop_mapper()
		{
			lock();
			m_req_stop_mapper = true;
			m_abort_ba_mapper = true;
			unlock();
		}

		bool is_req_stop_mapper()
		{
			return m_req_stop_mapper;
		}

		bool is_accept_kf_mapper()
		{
			return m_accept_kf_mapper;
		}

		void set_accept_kf_mapper(bool v)
		{
			lock();
			m_accept_kf_mapper = v;
			unlock();
		}

		bool is_not_stop()
		{
			lock();
			bool v = m_not_stop_mapper;
			unlock();
			return v;
		}

		bool set_not_stop(const bool v)
		{
			lock();

			if (v && m_stop_mapper){
				unlock();
				return false;
			}

			m_not_stop_mapper = v;

			unlock();
			return true;
		}

		bool is_abort_ba_mapper()
		{
			lock();
			bool v = m_abort_ba_mapper;
			unlock();
			return v;
		}

		bool * export_abort_ba_flag_mapper()
		{
			return &m_abort_ba_mapper;
		}

		void set_abort_ba_mapper(const bool v = true)
		{
			lock();
			m_abort_ba_mapper = v;
			unlock();
		}

		bool is_rst_loop_closer()
		{
			lock();
			bool v = m_rst_loop_closer;
			unlock();
			return v;
		}

		void rst_done_loop_closer()
		{
			lock();
			m_rst_loop_closer = false;
			unlock();
		}

		bool is_running_gba(){
			lock();
			bool v = m_running_gba;
			unlock();
			return v;
		}

		void set_running_gba(const bool v)
		{
			lock();
			m_running_gba = v;
			unlock();
		}

		bool is_stop_gba(){
			lock();
			bool v = m_stop_gba;
			unlock();
			return v;
		}

		bool * export_stop_gba(){
			return &m_stop_gba;
		}

		void set_stop_gba(const bool v)
		{
			lock();
			m_stop_gba = v;
			unlock();
		}

		bool is_finish_gba(){
			lock();
			bool v = m_finish_gba;
			unlock();
			return v;
		}

		void set_finish_gba(const bool v)
		{
			lock();
			m_finish_gba = v;
			unlock();
		}
	};

	class ch_map : public ch_base, public Map
	{
	public:
		ch_map(const char * name) :ch_base(name)
		{
		}
	};

	class ch_keyframe : public ch_base
	{
	protected:
		list<KeyFrame*> m_lkf;
	public:
		ch_keyframe(const char * name) : ch_base(name)
		{
		}

		bool empty(){
			lock();
			bool b = m_lkf.empty();
			unlock();
			return b;
		}

		void push(KeyFrame * pKF)
		{
			lock();
			m_lkf.push_back(pKF);
			unlock();
		}

		KeyFrame * pop()
		{
			lock();
			KeyFrame * pKF = m_lkf.front();
			m_lkf.pop_front();
			unlock();
			return pKF;
		}

		void clear()
		{
			lock();
			m_lkf.clear();
			unlock();
		}

		void clear_with_del()
		{
			lock();
			for (list<KeyFrame*>::iterator lit = m_lkf.begin();
				lit != m_lkf.end(); lit++)
				delete *lit;
			m_lkf.clear();
			unlock();
		}
	};

	class ch_keyframeDB : public ch_base, public KeyFrameDatabase
	{
	protected:
		ORBVocabulary * m_pvoc;
	public:
		ch_keyframeDB(const char * name) : ch_base(name), m_pvoc(NULL)
		{
		}
		~ch_keyframeDB()
		{
			m_pvoc = NULL;
		}

		bool load_voc(ORBVocabulary * pvoc)
		{
			lock();
			m_pvoc = pvoc;
			KeyFrameDatabase::init(*m_pvoc);
			unlock();
			return true;
		}

		ORBVocabulary * get_orb_voc()
		{
			lock();
			ORBVocabulary * pvoc = m_pvoc;
			unlock();
			return pvoc;
		}
	};

	class ch_trj : public ch_base
	{
	protected:
		list<Mat> m_Tcr;
		list<KeyFrame*> m_rkf;
		list<long long> m_tfrm;
		list<bool> m_lost;
	public:
		ch_trj(const char * name) :ch_base(name)
		{

		}
		virtual ~ch_trj(){
			m_Tcr.clear();
			m_rkf.clear();
			m_tfrm.clear();
			m_lost.clear();
		}

		void push(Mat & Tcr, KeyFrame * pkf, long long tfrm, bool blost){
			lock();
			m_Tcr.push_back(Tcr);
			m_rkf.push_back(pkf);
			m_tfrm.push_back(tfrm);
			m_lost.push_back(blost);
			unlock();
		}

		void clear()
		{
			lock();
			m_Tcr.clear();
			m_rkf.clear();
			m_tfrm.clear();
			m_lost.clear();
			unlock();
		}
	};

	class ch_frm : public ch_base
	{
	protected:
		Mat m_img;
		int m_num_keys;
		vector<bool> m_bmap;
		vector<bool> m_bvo;
		vector<KeyPoint> m_cur_keys, m_ini_keys;
		vector<int> m_ini_matches;
		e_tracking_state m_state;
		Mat m_Tcw;
	public:
		ch_frm(const char * name) : ch_base(name), m_state(SYSTEM_NOT_READY)
		{
		}

		// call if tracking state is NOT_INITIALIZED
		void init(Mat & img, vector<KeyPoint> & cur_keys, vector<KeyPoint> & ini_keys, vector<int> &ini_matches, e_tracking_state state)
		{
			lock();
			img.copyTo(m_img);
			m_cur_keys = cur_keys;
			m_num_keys = cur_keys.size();
			m_bmap = vector<bool>(m_num_keys, false);
			m_bvo = vector<bool>(m_num_keys, false);
			if (state == NOT_INITIALIZED){
				m_ini_keys = ini_keys;
				m_ini_matches = ini_matches;
			}
			m_state = state;
			unlock();
		}

		// call if tracking state is not NOT_INITIALIZED
		void update(Mat & img, vector<MapPoint*> &cur_mps, vector<bool> & outlier, vector<KeyPoint> & cur_keys, e_tracking_state state)
		{
			lock();
			img.copyTo(m_img);
			m_cur_keys = cur_keys;
			m_num_keys = cur_keys.size();
			m_bmap = vector<bool>(m_num_keys, false);
			m_bvo = vector<bool>(m_num_keys, false);
			m_state = state;
			if (m_state == OK){
				for (int i = 0; i<m_num_keys; i++)
				{
					MapPoint* pMP = cur_mps[i];
					if (pMP)
					{
						if (!outlier[i])
						{
							if (pMP->Observations()>0)
								m_bmap[i] = true;
							else
								m_bvo[i] = true;
						}
					}
				}
			}
			unlock();
		}

		void get(Mat & img, vector<int> & matches, vector<KeyPoint> & ini_keys,
			vector<KeyPoint> & cur_keys, vector<bool> & bvo, vector<bool> & bmap,
			e_tracking_state & state)
		{
			lock();
			if (m_state == SYSTEM_NOT_READY)
				m_state = NO_IMAGES_YET;

			m_img.copyTo(img);

			if (m_state == NOT_INITIALIZED){
				cur_keys = m_cur_keys;
				ini_keys = m_ini_keys;
				matches = m_ini_matches;
			}
			else if (m_state == OK){
				cur_keys = m_cur_keys;
				bvo = m_bvo;
				bmap = m_bmap;
			}
			else if (m_state == LOST){
				cur_keys = m_cur_keys;
			}
			state = m_state;
			unlock();
		}

		void set_camera_pose(Mat & Tcw)
		{
			lock();
			m_Tcw = Tcw.clone();
			unlock();
		}

		Mat get_camera_pose()
		{
			lock();
			Mat Tcw = m_Tcw;
			unlock();
			return Tcw;
		}
	};
}

#endif
