#include "stdafx.h"
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

#include <cstdio>
#include <cstring>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <thread>
#include <mutex>

using namespace std;

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;


#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <GL/glut.h>
#include <GL/glu.h>


#include"../orb_slam/PnPsolver.h"
#include "../orb_slam/ORBmatcher.h"
#include "../orb_slam/Converter.h"
#include "../orb_slam/Sim3Solver.h"
#include "f_glfw_window.h"
#include "f_orb_slam.h"

namespace ORB_SLAM2
{

	///////////////////////////////////////////////// tracker
	f_tracker::f_tracker(const char * name) :f_base(name),
		m_cam(NULL), m_pORBEx(NULL), m_pORBExIni(NULL),
		m_kf_mapper(NULL), m_kfdb(NULL), m_map(NULL), m_trj(NULL), m_frm(NULL),
		m_max_frms(30), m_min_frms(0),
		m_pvoc(NULL), m_timg(-1), m_ifrm(-1), m_state(NO_IMAGES_YET), m_last_state(NO_IMAGES_YET),
		m_ifrm_last_reloc(0), m_pref_kf(NULL), m_bvo(false), m_pinit(NULL), m_plast_kf(NULL), m_last_kf_id(-1),
		m_num_matches_inliers(0)
	{
		register_fpar("ch_sys", (ch_base**)&m_sys, typeid(ch_sys).name(), "System channel.");
		register_fpar("ch_cam", (ch_base**)&m_cam, typeid(ch_image_ref).name(), "Camera image channel.");
		register_fpar("ch_kfdb", (ch_base**)&m_kfdb, typeid(ch_keyframeDB).name(), "Keyframe Database channel");
		register_fpar("ch_kf_mapper", (ch_base**)&m_kf_mapper, typeid(ch_keyframe).name(), "Keyframe channel pushed to the local mapper");
		register_fpar("ch_map", (ch_base**)&m_map, typeid(ch_map).name(), "Map channel.");
		register_fpar("ch_trj", (ch_base**)&m_trj, typeid(ch_trj).name(), "Trajectory channel.");
		register_fpar("ch_frm", (ch_base**)&m_frm, typeid(ch_frm).name(), "Frame channel.");

		m_fcp[0] = m_fvoc[0] = '\0';
		register_fpar("fcp", m_fcp, 1024, "File of camera parameters.");
		register_fpar("fvoc", m_fvoc, 1024, "File of vocablary.");

		register_fpar("max_frms", &m_max_frms, "Maximum frames for new key frame insertion");
		register_fpar("min_frms", &m_min_frms, "Minimum frames for preventing new key frame insertion");
		// for orb extractor
		register_fpar("num_features", &m_num_features, "Number of ORB features.");
		register_fpar("scale_factor", &m_scale_factor, "Scale factor of ORB extractor");
		register_fpar("num_levels", &m_num_levels, "Number of pyramid levels of ORB extractor");
		register_fpar("th_fast_ini", &m_th_fast_ini, "Ini Threshold for FAST extraction of ORB Extractor");
		register_fpar("th_fast_min", &m_th_fast_min, "Min Threshold for FAST extraction of ORB Extractor");
	}

	f_tracker::~f_tracker()
	{
	}

	bool f_tracker::init_run()
	{
		if (!m_cp.read(m_fcp)){
			cerr << "Failed to load camera parameter in f_tracker::init_run" << endl;
			return false;
		}

		m_pORBEx = new ORBextractor(m_num_features, m_scale_factor, m_num_levels, m_th_fast_ini, m_th_fast_min);
		if (m_pORBEx){
			cerr << "Failed to allocate ORBEx in f_tracker::init_run" << endl;
			return false;
		}

		m_pORBExIni = new ORBextractor(2 * m_num_features, m_scale_factor, m_num_levels, m_th_fast_ini, m_th_fast_min);
		if (m_pORBExIni){
			cerr << "Failed to allocate ORBExIni in f_tracker::init_run" << endl;
			return false;
		}

		if (m_sys){
			cerr << "System channel is not found." << endl;
			return false;
		}

		if (m_pvoc)
			delete m_pvoc;

		m_pvoc = new ORBVocabulary;
		if (!m_pvoc->loadFromTextFile(m_fvoc)){
			delete m_pvoc;
			m_pvoc = NULL;
			return false;
		}

		if (!m_kfdb || !m_kfdb->load_voc(m_pvoc)){
			cerr << "Failed to initialize Keyframe Database in f_tracker::ini_run" << endl;
			return false;
		}

		return true;
	}

	void f_tracker::destroy_run()
	{
		if (m_pORBEx){
			delete m_pORBEx;
			m_pORBEx = NULL;
		}

		if (m_pORBExIni){
			delete m_pORBExIni;
			m_pORBExIni = NULL;
		}
	}

	bool f_tracker::proc()
	{
		if (m_sys->is_rst_tracker()){
			reset();
		}

		if (m_sys->is_rst_mapper() || m_sys->is_rst_loop_closer()){
			return true;
		}

		if (!load_frm())
			return true;
	
		if (m_state == NO_IMAGES_YET)
			m_state = NOT_INITIALIZED;

		m_last_state = m_state;

		unique_lock<mutex> lock(m_map->mMutexMapUpdate);
	
		if (m_state == NOT_INITIALIZED)
		{
			init_slam();

			if (m_frm){
				if (m_state == NOT_INITIALIZED)
					m_frm->init(m_img, m_cur_frm.mvKeys, m_ini_frm.mvKeys, m_ini_matches, m_state);
				else
					m_frm->update(m_img, m_cur_frm.mvpMapPoints, m_cur_frm.mvbOutlier, m_cur_frm.mvKeys, m_state);
			}

			if (m_state != OK){
				return true;
			}
	
		}
		else{
			bool bOK;
			if (m_state == OK){
				update_last_frm_mps();

				if (m_vel.empty() || m_cur_frm.mnId < m_ifrm_last_reloc + 2){
					bOK = track_rkf();
				}
				else{
					bOK = track_mm();
					if (!bOK)
						bOK = track_rkf();
				}
			}
			else{
				bOK = reloc();
			}

			m_cur_frm.mpReferenceKF = m_pref_kf;

			if (bOK && !m_bvo)
				bOK = track_local_map();

			if (bOK){
				m_state = OK;
			}
			else{
				m_state = LOST;
			}

			if (m_frm){
				m_frm->update(m_img, m_cur_frm.mvpMapPoints, m_cur_frm.mvbOutlier, m_cur_frm.mvKeys, m_state);
			}

			if (bOK){

				// update motion model
				if (!m_last_frm.mTcw.empty()){
					Mat LastTwc = Mat::eye(4, 4, CV_32F);
					m_last_frm.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
					m_last_frm.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
				}
				else{
					m_vel = Mat();
				}

				if (m_frm){
					m_frm->set_camera_pose(m_cur_frm.mTcw);
				}

				// clean temporal point matches
				for (int i = 0; i<m_cur_frm.N; i++)
				{
					MapPoint* pMP = m_cur_frm.mvpMapPoints[i];
					if (pMP)
					if (pMP->Observations()<1)
					{
						m_cur_frm.mvbOutlier[i] = false;
						m_cur_frm.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
					}
				}

				// delete temporal MapPoints
				for (list<MapPoint*>::iterator lit = m_tmp_mps.begin(), lend = m_tmp_mps.end(); lit != lend; lit++)
				{
					MapPoint* pMP = *lit;
					delete pMP;
				}
				m_tmp_mps.clear();

				if (need_new_kf()){
					create_new_kf();
				}

				// Outliers during tracking maybe used in BA, but now we discard them as tracking targets.
				for (int i = 0; i<m_cur_frm.N; i++)
				{
					if (m_cur_frm.mvpMapPoints[i] && m_cur_frm.mvbOutlier[i])
						m_cur_frm.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
				}

			}

			if (m_state = LOST){
				if (m_map->KeyFramesInMap() <= 5){
					cout << "Track lost soon after initialisation, reseting." << endl;
					m_sys->set_rst();
					return true;
				}
			}

			if (!m_cur_frm.mpReferenceKF)
				m_cur_frm.mpReferenceKF = m_pref_kf;

			m_last_frm = Frame(m_cur_frm);
		}

		KeyFrame * pkf_trj = NULL;
		long long tfrm_trj = -1;
		if (!m_cur_frm.mTcw.empty()){
			m_Tlr = m_cur_frm.mTcw * m_cur_frm.mpReferenceKF->GetPoseInverse();
			pkf_trj = m_cur_frm.mpReferenceKF;
			tfrm_trj = m_cur_frm.mTimeStamp;
		}

		if (m_trj){
			m_trj->push(m_Tlr, pkf_trj, tfrm_trj, m_state == LOST);
		}
		
		return true;
	}

	bool f_tracker::load_frm()
	{
		// load new image and create frame object
		if (!m_cam){
			cerr << "Camera channel is not found." << endl;
			return false;
		}

		long long t, ifrm;
		Mat img = m_cam->get_img(t, ifrm);

		if (img.empty())
			return false;

		if (t <= m_timg || ifrm <= m_ifrm)
			return false;

		m_timg = t;
		m_ifrm = ifrm;
		if (img.channels() != 3)
			cvtColor(img, m_img, CV_BGR2GRAY);
		else
			m_img = img.clone();

		float bf = 0.f, th_depth = 0.f; // these parameters are not used in monocular mode, and should be eliminated later.
		if (m_state == NOT_INITIALIZED || m_state == NO_IMAGES_YET)
			m_cur_frm = Frame(m_img, m_timg, m_pORBExIni, m_pvoc, m_cp.getCvPrjMat(), m_cp.getCvDistMat(), bf, th_depth);
		else
			m_cur_frm = Frame(m_img, m_timg, m_pORBEx, m_pvoc, m_cp.getCvPrjMat(), m_cp.getCvDistMat(), bf, th_depth);


		return true;
	}

	void f_tracker::update_last_frm_mps()
	{
		for (int i = 0; i<m_last_frm.N; i++)
		{
			MapPoint* pMP = m_last_frm.mvpMapPoints[i];

			if (pMP)
			{
				MapPoint* pRep = pMP->GetReplaced();
				if (pRep)
				{
					m_last_frm.mvpMapPoints[i] = pRep;
				}
			}
		}
	}

	void f_tracker::init_slam()
	{
		if (!m_pinit)
		{
			// Set Reference Frame
			if (m_cur_frm.mvKeys.size()>100)
			{
				m_ini_frm = Frame(m_cur_frm);
				m_last_frm = Frame(m_cur_frm);
				m_prev_matched_p2d.resize(m_cur_frm.mvKeysUn.size());
				for (size_t i = 0; i<m_cur_frm.mvKeysUn.size(); i++)
					m_prev_matched_p2d[i] = m_cur_frm.mvKeysUn[i].pt;

				if (m_pinit)
					delete m_pinit;

				m_pinit = new Initializer(m_cur_frm, 1.0, 200);

				fill(m_ini_matches.begin(), m_ini_matches.end(), -1);

				return;
			}
		}
		else
		{
			// Try to initialize
			if ((int)m_cur_frm.mvKeys.size() <= 100)
			{
				delete m_pinit;
				m_pinit = static_cast<Initializer*>(NULL);
				fill(m_ini_matches.begin(), m_ini_matches.end(), -1);
				return;
			}

			// Find correspondences
			ORBmatcher matcher(0.9, true);
			int nmatches = matcher.SearchForInitialization(m_ini_frm, m_cur_frm, m_prev_matched_p2d, m_ini_matches, 100);

			// Check if there are enough correspondences
			if (nmatches<100)
			{
				delete m_pinit;
				m_pinit = static_cast<Initializer*>(NULL);
				return;
			}

			cv::Mat Rcw; // Current Camera Rotation
			cv::Mat tcw; // Current Camera Translation
			vector<bool> vbTriangulated; // Triangulated Correspondences (m_ini_matches)

			if (m_pinit->Initialize(m_cur_frm, m_ini_matches, Rcw, tcw, m_ini_p3d, vbTriangulated))
			{
				for (size_t i = 0, iend = m_ini_matches.size(); i<iend; i++)
				{
					if (m_ini_matches[i] >= 0 && !vbTriangulated[i])
					{
						m_ini_matches[i] = -1;
						nmatches--;
					}
				}

				// Set Frame Poses
				m_ini_frm.SetPose(cv::Mat::eye(4, 4, CV_32F));
				cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
				Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
				tcw.copyTo(Tcw.rowRange(0, 3).col(3));
				m_cur_frm.SetPose(Tcw);

				create_map();
			}
		}
	}

	void f_tracker::create_map()
	{
		// Create KeyFrames
		KeyFrame* pKFini = new KeyFrame(m_ini_frm, m_map, m_kfdb);
		KeyFrame* pKFcur = new KeyFrame(m_cur_frm, m_map, m_kfdb);

		pKFini->ComputeBoW();
		pKFcur->ComputeBoW();

		// Insert KFs in the map
		m_map->AddKeyFrame(pKFini);
		m_map->AddKeyFrame(pKFcur);

		// Create MapPoints and asscoiate to keyframes
		for (size_t i = 0; i<m_ini_matches.size(); i++)
		{
			if (m_ini_matches[i]<0)
				continue;

			//Create MapPoint.
			cv::Mat worldPos(m_ini_p3d[i]);

			MapPoint* pMP = new MapPoint(worldPos, pKFcur, m_map);

			pKFini->AddMapPoint(pMP, i);
			pKFcur->AddMapPoint(pMP, m_ini_matches[i]);

			pMP->AddObservation(pKFini, i);
			pMP->AddObservation(pKFcur, m_ini_matches[i]);

			pMP->ComputeDistinctiveDescriptors();
			pMP->UpdateNormalAndDepth();

			//Fill Current Frame structure
			m_cur_frm.mvpMapPoints[m_ini_matches[i]] = pMP;
			m_cur_frm.mvbOutlier[m_ini_matches[i]] = false;

			//Add to Map
			m_map->AddMapPoint(pMP);
		}

		// Update Connections
		pKFini->UpdateConnections();
		pKFcur->UpdateConnections();

		// Bundle Adjustment
		cout << "New Map created with " << m_map->MapPointsInMap() << " points" << endl;

		Optimizer::GlobalBundleAdjustemnt(m_map, 20);

		// Set median depth to 1
		float medianDepth = pKFini->ComputeSceneMedianDepth(2);
		float invMedianDepth = 1.0f / medianDepth;

		if (medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
		{
			cout << "Wrong initialization, reseting..." << endl;
			m_sys->set_rst();
			return;
		}

		// Scale initial baseline
		cv::Mat Tc2w = pKFcur->GetPose();
		Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3)*invMedianDepth;
		pKFcur->SetPose(Tc2w);

		// Scale points
		vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
		for (size_t iMP = 0; iMP<vpAllMapPoints.size(); iMP++)
		{
			if (vpAllMapPoints[iMP])
			{
				MapPoint* pMP = vpAllMapPoints[iMP];
				pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
			}
		}

		m_kf_mapper->push(pKFcur);
		m_kf_mapper->push(pKFini);

		m_cur_frm.SetPose(pKFcur->GetPose());
		m_last_kf_id = m_cur_frm.mnId;
		m_plast_kf = pKFcur;

		m_local_kfs.push_back(pKFcur);
		m_local_kfs.push_back(pKFini);
		m_local_mps = m_map->GetAllMapPoints();
		m_pref_kf = pKFcur;
		m_cur_frm.mpReferenceKF = pKFcur;

		m_last_frm = Frame(m_cur_frm);

		m_map->SetReferenceMapPoints(m_local_mps);

		m_map->mvpKeyFrameOrigins.push_back(pKFini);

		m_state = OK;
	}

	void f_tracker::reset()
	{
		// Clear BoW Database
		if (m_kfdb){
			m_kfdb->clear();
		}
		// Clear Map (this erase MapPoints and KeyFrames)

		if (m_map){
			m_map->clear();
		}

		KeyFrame::nNextId = 0;
		Frame::nNextId = 0;
		m_state = NO_IMAGES_YET;

		if (m_pinit)
		{
			delete m_pinit;
			m_pinit = static_cast<Initializer*>(NULL);
		}

		if (m_trj)
			m_trj->clear();
	}

	bool f_tracker::need_new_kf()
	{
		// If Local Mapping is freezed by a Loop Closure do not insert keyframes
		if (m_sys && (m_sys->is_stopped_mapper() || m_sys->is_req_stop_mapper()))
			return false;

		const int nKFs = m_map->KeyFramesInMap();

		// Do not insert keyframes if not enough frames have passed from last relocalisation
		if (m_cur_frm.mnId< m_last_kf_id + m_max_frms && nKFs > m_max_frms)
			return false;

		// Tracked MapPoints in the reference keyframe
		int nMinObs = 3;
		if (nKFs <= 2)
			nMinObs = 2;
		int nRefMatches = m_pref_kf->TrackedMapPoints(nMinObs);

		// Local Mapping accept keyframes?
		bool bLocalMappingIdle = (m_sys != NULL && m_sys->is_accept_kf_mapper());
		// "total matches = matches to map + visual odometry matches"
		// Visual odometry matches will become MapPoints if we insert a keyframe.
		// This ratio measures how many MapPoints we could create if we insert a keyframe.
		int nMap = 1;
		int nTotal = 1;

		const float ratioMap = (float)nMap / fmax(1.0f, nTotal);

		// Thresholds
		float thRefRatio = 0.9f;
		float thMapRatio = 0.35f;
		if (m_num_matches_inliers > 300)
			thMapRatio = 0.20f;

		// Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
		const bool c1a = m_cur_frm.mnId >= m_last_kf_id + m_max_frms;
		// Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
		const bool c1b = (m_cur_frm.mnId >= m_last_kf_id + m_min_frms && bLocalMappingIdle);
		// Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
		const bool c2 = ((m_num_matches_inliers < nRefMatches*thRefRatio || ratioMap<thMapRatio) 
			&& m_num_matches_inliers > 15);

		if ((c1a || c1b) && c2)
		{
			// If the mapping accepts keyframes, insert keyframe.
			// Otherwise send a signal to interrupt BA
			if (bLocalMappingIdle)
			{
				return true;
			}else
			{
				if (m_sys)
					m_sys->set_abort_ba_mapper();
				return false;
			}
		}
		else
			return false;
	}

	void f_tracker::create_new_kf()
	{
		// if local mapper has already been stopped, it means that the GBA requested 
		// preventing the new key frame insertion.
		if (!m_sys->set_not_stop(true))
			return;

		KeyFrame* pKF = new KeyFrame(m_cur_frm, m_map, m_kfdb);

		m_pref_kf = pKF;
		m_cur_frm.mpReferenceKF = pKF;

		m_kf_mapper->push(pKF);

		m_sys->set_not_stop(false);

		m_last_kf_id = m_cur_frm.mnId;
		m_plast_kf = pKF;
	}

	bool f_tracker::track_rkf()
	{
		// Compute Bag of Words vector
		m_cur_frm.ComputeBoW();

		// We perform first an ORB matching with the reference keyframe
		// If enough matches are found we setup a PnP solver
		ORBmatcher matcher(0.7, true);
		vector<MapPoint*> vpMapPointMatches;

		int nmatches = matcher.SearchByBoW(m_pref_kf, m_cur_frm, vpMapPointMatches);

		if (nmatches<15)
			return false;

		m_cur_frm.mvpMapPoints = vpMapPointMatches;
		m_cur_frm.SetPose(m_last_frm.mTcw);

		Optimizer::PoseOptimization(&m_cur_frm);

		// Discard outliers
		int nmatchesMap = 0;
		for (int i = 0; i<m_cur_frm.N; i++)
		{
			if (m_cur_frm.mvpMapPoints[i])
			{
				if (m_cur_frm.mvbOutlier[i])
				{
					MapPoint* pMP = m_cur_frm.mvpMapPoints[i];

					m_cur_frm.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
					m_cur_frm.mvbOutlier[i] = false;
					pMP->mbTrackInView = false;
					pMP->mnLastFrameSeen = m_cur_frm.mnId;
					nmatches--;
				}
				else if (m_cur_frm.mvpMapPoints[i]->Observations()>0)
					nmatchesMap++;
			}
		}

		return nmatchesMap >= 10;
	}

	bool f_tracker::track_mm()
	{
		ORBmatcher matcher(0.9, true);

		// Update last frame pose according to its reference keyframe
		// Create "visual odometry" points
		update_last_frm();

		m_cur_frm.SetPose(m_vel * m_last_frm.mTcw);

		fill(m_cur_frm.mvpMapPoints.begin(), m_cur_frm.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));

		// Project points seen in previous frame
		int th = 15;
		int nmatches = matcher.SearchByProjection(m_cur_frm, m_last_frm, th, true);

		// If few matches, uses a wider window search
		if (nmatches<20)
		{
			fill(m_cur_frm.mvpMapPoints.begin(), m_cur_frm.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
			nmatches = matcher.SearchByProjection(m_cur_frm, m_last_frm, 2 * th, true);
		}

		if (nmatches<20)
			return false;

		// Optimize frame pose with all matches
		Optimizer::PoseOptimization(&m_cur_frm);

		// Discard outliers
		int nmatchesMap = 0;
		for (int i = 0; i<m_cur_frm.N; i++)
		{
			if (m_cur_frm.mvpMapPoints[i])
			{
				if (m_cur_frm.mvbOutlier[i])
				{
					MapPoint* pMP = m_cur_frm.mvpMapPoints[i];

					m_cur_frm.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
					m_cur_frm.mvbOutlier[i] = false;
					pMP->mbTrackInView = false;
					pMP->mnLastFrameSeen = m_cur_frm.mnId;
					nmatches--;
				}
				else if (m_cur_frm.mvpMapPoints[i]->Observations()>0)
					nmatchesMap++;
			}
		}

		return nmatchesMap >= 10;
	}

	void f_tracker::update_last_frm()
	{
		// Update pose according to reference keyframe
		KeyFrame* pRef = m_last_frm.mpReferenceKF;
		m_last_frm.SetPose(m_Tlr*pRef->GetPose());
	}

	bool f_tracker::track_local_map()
	{
		// We have an estimation of the camera pose and some map points tracked in the frame.
		// We retrieve the local map and try to find matches to points in the local map.

		update_local_map();
		search_local_points();

		// Optimize Pose
		Optimizer::PoseOptimization(&m_cur_frm);
		m_num_matches_inliers = 0;

		// Update MapPoints Statistics
		for (int i = 0; i<m_cur_frm.N; i++)
		{
			if (m_cur_frm.mvpMapPoints[i])
			{
				if (!m_cur_frm.mvbOutlier[i])
				{
					m_cur_frm.mvpMapPoints[i]->IncreaseFound();
					if (m_cur_frm.mvpMapPoints[i]->Observations()>0)
						m_num_matches_inliers++;
				}
			}
		}

		// Decide if the tracking was succesful
		// More restrictive if there was a relocalization recently
		if (m_cur_frm.mnId<m_ifrm_last_reloc + m_max_frms && m_num_matches_inliers < 50)
			return false;

		if (m_num_matches_inliers < 30)
			return false;
		else
			return true;
	}

	void f_tracker::update_local_map()
	{
		m_map->SetReferenceMapPoints(m_local_mps);

		update_local_kfs();
		update_local_pts();
	}

	void f_tracker::update_local_kfs()
	{
		// Each map point vote for the keyframes in which it has been observed
		map<KeyFrame*, int> keyframeCounter;
		for (int i = 0; i<m_cur_frm.N; i++)
		{
			if (m_cur_frm.mvpMapPoints[i])
			{
				MapPoint* pMP = m_cur_frm.mvpMapPoints[i];
				if (!pMP->isBad())
				{
					const map<KeyFrame*, size_t> observations = pMP->GetObservations();
					for (map<KeyFrame*, size_t>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
						keyframeCounter[it->first]++;
				}
				else
				{
					m_cur_frm.mvpMapPoints[i] = NULL;
				}
			}
		}

		if (keyframeCounter.empty())
			return;

		int max = 0;
		KeyFrame* pKFmax = static_cast<KeyFrame*>(NULL);

		m_local_kfs.clear();
		m_local_kfs.reserve(3 * keyframeCounter.size());

		// All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
		for (map<KeyFrame*, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++)
		{
			KeyFrame* pKF = it->first;

			if (pKF->isBad())
				continue;

			if (it->second>max)
			{
				max = it->second;
				pKFmax = pKF;
			}

			m_local_kfs.push_back(it->first);
			pKF->mnTrackReferenceForFrame = m_cur_frm.mnId;
		}


		// Include also some not-already-included keyframes that are neighbors to already-included keyframes
		for (vector<KeyFrame*>::const_iterator itKF = m_local_kfs.begin(), itEndKF = m_local_kfs.end(); itKF != itEndKF; itKF++)
		{
			// Limit the number of keyframes
			if (m_local_kfs.size()>80)
				break;

			KeyFrame* pKF = *itKF;

			const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

			for (vector<KeyFrame*>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF; itNeighKF++)
			{
				KeyFrame* pNeighKF = *itNeighKF;
				if (!pNeighKF->isBad())
				{
					if (pNeighKF->mnTrackReferenceForFrame != m_cur_frm.mnId)
					{
						m_local_kfs.push_back(pNeighKF);
						pNeighKF->mnTrackReferenceForFrame = m_cur_frm.mnId;
						break;
					}
				}
			}

			const set<KeyFrame*> spChilds = pKF->GetChilds();
			for (set<KeyFrame*>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++)
			{
				KeyFrame* pChildKF = *sit;
				if (!pChildKF->isBad())
				{
					if (pChildKF->mnTrackReferenceForFrame != m_cur_frm.mnId)
					{
						m_local_kfs.push_back(pChildKF);
						pChildKF->mnTrackReferenceForFrame = m_cur_frm.mnId;
						break;
					}
				}
			}

			KeyFrame* pParent = pKF->GetParent();
			if (pParent)
			{
				if (pParent->mnTrackReferenceForFrame != m_cur_frm.mnId)
				{
					m_local_kfs.push_back(pParent);
					pParent->mnTrackReferenceForFrame = m_cur_frm.mnId;
					break;
				}
			}

		}

		if (pKFmax)
		{
			m_pref_kf = pKFmax;
			m_cur_frm.mpReferenceKF = m_pref_kf;
		}
	}

	void f_tracker::update_local_pts()
	{
		m_local_mps.clear();

		for (vector<KeyFrame*>::const_iterator itKF = m_local_kfs.begin(), itEndKF = m_local_kfs.end(); itKF != itEndKF; itKF++)
		{
			KeyFrame* pKF = *itKF;
			const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

			for (vector<MapPoint*>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++)
			{
				MapPoint* pMP = *itMP;
				if (!pMP)
					continue;
				if (pMP->mnTrackReferenceForFrame == m_cur_frm.mnId)
					continue;
				if (!pMP->isBad())
				{
					m_local_mps.push_back(pMP);
					pMP->mnTrackReferenceForFrame = m_cur_frm.mnId;
				}
			}
		}
	}

	void f_tracker::search_local_points()
	{
		// Do not search map points already matched
		for (vector<MapPoint*>::iterator vit = m_cur_frm.mvpMapPoints.begin(), vend = m_cur_frm.mvpMapPoints.end(); vit != vend; vit++)
		{
			MapPoint* pMP = *vit;
			if (pMP)
			{
				if (pMP->isBad())
				{
					*vit = static_cast<MapPoint*>(NULL);
				}
				else
				{
					pMP->IncreaseVisible();
					pMP->mnLastFrameSeen = m_cur_frm.mnId;
					pMP->mbTrackInView = false;
				}
			}
		}

		int nToMatch = 0;

		// Project points in frame and check its visibility
		for (vector<MapPoint*>::iterator vit = m_local_mps.begin(), vend = m_local_mps.end(); vit != vend; vit++)
		{
			MapPoint* pMP = *vit;
			if (pMP->mnLastFrameSeen == m_cur_frm.mnId)
				continue;
			if (pMP->isBad())
				continue;
			// Project (this fills MapPoint variables for matching)
			if (m_cur_frm.isInFrustum(pMP, 0.5))
			{
				pMP->IncreaseVisible();
				nToMatch++;
			}
		}

		if (nToMatch>0)
		{
			ORBmatcher matcher(0.8);
			int th = 1;
			// If the camera has been relocalised recently, perform a coarser search
			if (m_cur_frm.mnId < m_ifrm_last_reloc + 2)
				th = 5;
			
			matcher.SearchByProjection(m_cur_frm, m_local_mps, th);
		}
	}

	bool f_tracker::reloc()
	{
		// Compute Bag of Words Vector
		m_cur_frm.ComputeBoW();

		// Relocalization is performed when tracking is lost
		// Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
		vector<KeyFrame*> vpCandidateKFs = m_kfdb->DetectRelocalizationCandidates(&m_cur_frm);

		if (vpCandidateKFs.empty())
			return false;

		const int nKFs = vpCandidateKFs.size();

		// We perform first an ORB matching with each candidate
		// If enough matches are found we setup a PnP solver
		ORBmatcher matcher(0.75, true);

		vector<PnPsolver*> vpPnPsolvers;
		vpPnPsolvers.resize(nKFs);

		vector<vector<MapPoint*> > vvpMapPointMatches;
		vvpMapPointMatches.resize(nKFs);

		vector<bool> vbDiscarded;
		vbDiscarded.resize(nKFs);

		int nCandidates = 0;

		for (int i = 0; i<nKFs; i++)
		{
			KeyFrame* pKF = vpCandidateKFs[i];
			if (pKF->isBad())
				vbDiscarded[i] = true;
			else
			{
				int nmatches = matcher.SearchByBoW(pKF, m_cur_frm, vvpMapPointMatches[i]);
				if (nmatches<15)
				{
					vbDiscarded[i] = true;
					continue;
				}
				else
				{
					PnPsolver* pSolver = new PnPsolver(m_cur_frm, vvpMapPointMatches[i]);
					pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
					vpPnPsolvers[i] = pSolver;
					nCandidates++;
				}
			}
		}

		// Alternatively perform some iterations of P4P RANSAC
		// Until we found a camera pose supported by enough inliers
		bool bMatch = false;
		ORBmatcher matcher2(0.9, true);

		while (nCandidates>0 && !bMatch)
		{
			for (int i = 0; i<nKFs; i++)
			{
				if (vbDiscarded[i])
					continue;

				// Perform 5 Ransac Iterations
				vector<bool> vbInliers;
				int nInliers;
				bool bNoMore;

				PnPsolver* pSolver = vpPnPsolvers[i];
				cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

				// If Ransac reachs max. iterations discard keyframe
				if (bNoMore)
				{
					vbDiscarded[i] = true;
					nCandidates--;
				}

				// If a Camera Pose is computed, optimize
				if (!Tcw.empty())
				{
					Tcw.copyTo(m_cur_frm.mTcw);

					set<MapPoint*> sFound;

					const int np = vbInliers.size();

					for (int j = 0; j<np; j++)
					{
						if (vbInliers[j])
						{
							m_cur_frm.mvpMapPoints[j] = vvpMapPointMatches[i][j];
							sFound.insert(vvpMapPointMatches[i][j]);
						}
						else
							m_cur_frm.mvpMapPoints[j] = NULL;
					}

					int nGood = Optimizer::PoseOptimization(&m_cur_frm);

					if (nGood<10)
						continue;

					for (int io = 0; io<m_cur_frm.N; io++)
					if (m_cur_frm.mvbOutlier[io])
						m_cur_frm.mvpMapPoints[io] = static_cast<MapPoint*>(NULL);

					// If few inliers, search by projection in a coarse window and optimize again
					if (nGood<50)
					{
						int nadditional = matcher2.SearchByProjection(m_cur_frm, vpCandidateKFs[i], sFound, 10, 100);

						if (nadditional + nGood >= 50)
						{
							nGood = Optimizer::PoseOptimization(&m_cur_frm);

							// If many inliers but still not enough, search by projection again in a narrower window
							// the camera has been already optimized with many points
							if (nGood>30 && nGood<50)
							{
								sFound.clear();
								for (int ip = 0; ip<m_cur_frm.N; ip++)
								if (m_cur_frm.mvpMapPoints[ip])
									sFound.insert(m_cur_frm.mvpMapPoints[ip]);
								nadditional = matcher2.SearchByProjection(m_cur_frm, vpCandidateKFs[i], sFound, 3, 64);

								// Final optimization
								if (nGood + nadditional >= 50)
								{
									nGood = Optimizer::PoseOptimization(&m_cur_frm);

									for (int io = 0; io<m_cur_frm.N; io++)
									if (m_cur_frm.mvbOutlier[io])
										m_cur_frm.mvpMapPoints[io] = NULL;
								}
							}
						}
					}


					// If the pose is supported by enough inliers stop ransacs and continue
					if (nGood >= 50)
					{
						bMatch = true;
						break;
					}
				}
			}
		}

		if (!bMatch)
		{
			return false;
		}
		else
		{
			m_ifrm_last_reloc = m_cur_frm.mnId;
			return true;
		}
	}

	///////////////////////////////////////////////// local mapper
	f_local_mapper::f_local_mapper(const char * name) :f_base(name), m_kf_loop_closer(NULL), m_sys(NULL),
		m_kf_tracker(NULL), m_map(NULL), m_cur_kf(NULL)
	{
		register_fpar("ch_sys", (ch_base**)&m_sys, typeid(ch_sys).name(), "System channel.");
		register_fpar("ch_kf_tracker", (ch_base**)&m_kf_tracker, typeid(ch_keyframe).name(), "Key frame channel from tracker");
		register_fpar("ch_kf_loop_closer", (ch_base**)&m_kf_loop_closer, typeid(ch_keyframe).name(), "key frame channel from loop closer");
		register_fpar("ch_map", (ch_base**)&m_map, typeid(ch_map).name(), "Map chanenl");
	}

	f_local_mapper::~f_local_mapper()
	{
	}

	bool f_local_mapper::init_run()
	{
		if (!m_sys){
			cerr << "Channel sys not found in f_local_mapper::init_run." << endl;
			return false;
		}

		m_sys->set_stopped_mapper(false);
		m_sys->set_finished_mapper(false);
		
		if (!m_kf_tracker){
			cerr << "Channel kf_tracker not found in f_local_mapper:init_run." << endl;
			return false;
		}

		if(!m_map){
			cerr << "Channel map not found in f_local_mapper::init_run" << endl;
			return false;
		}

		return true;
	}

	void f_local_mapper::destroy_run()
	{
		if (m_sys){
			m_sys->set_stopped_mapper(true);
			m_sys->set_finished_mapper(false);
		}
	}

	bool f_local_mapper::proc()
	{
		// Tracking will see that Local Mapping is busy
		m_sys->set_accept_kf_mapper(false);

		// Check if there are keyframes in the queue
		if (!m_kf_tracker->empty())
		{
			// BoW conversion and insertion in Map
			proc_new_kf();

			// Check recent MapPoints
			cull_mps();

			// Triangulate new MapPoints
			create_new_mps();

			if (m_kf_tracker->empty())
			{
				// Find more matches in neighbor keyframes and fuse point duplications
				search_neighbours();
			}

			m_sys->set_abort_ba_mapper(false);

			if (m_kf_tracker->empty() && !m_sys->is_req_stop_mapper())
			{
				// Local BA
				if (m_map->KeyFramesInMap()>2)
					Optimizer::LocalBundleAdjustment(m_cur_kf, m_sys->export_abort_ba_flag_mapper(), m_map);

				// Check redundant local Keyframes
				cull_kfs();
			}

			m_kf_loop_closer->push(m_cur_kf);
		}
		else if (m_sys->stop_mapper())
		{
			while (m_sys->is_stopped_mapper() && !m_sys->is_finished_mapper()){
#ifdef _WIN32
				Sleep(3);
#else
				usleep(3000);
#endif
			}
			m_kf_tracker->clear_with_del();
		}

		if(m_sys->is_rst_mapper()){
			m_kf_tracker->clear();
			m_recent_mps.clear();
			m_sys->rst_done_mapper();
		}

		// Tracking will see that Local Mapping is busy
		m_sys->set_accept_kf_mapper(true);

		return true;
	}

	void f_local_mapper::proc_new_kf()
	{
		m_cur_kf = m_kf_tracker->pop();

		// Compute Bags of Words structures
		m_cur_kf->ComputeBoW();

		// Associate MapPoints to the new keyframe and update normal and descriptor
		const vector<MapPoint*> vpMapPointMatches = m_cur_kf->GetMapPointMatches();

		for (size_t i = 0; i<vpMapPointMatches.size(); i++)
		{
			MapPoint* pMP = vpMapPointMatches[i];
			if (pMP)
			{
				if (!pMP->isBad())
				{
					if (!pMP->IsInKeyFrame(m_cur_kf))
					{
						pMP->AddObservation(m_cur_kf, i);
						pMP->UpdateNormalAndDepth();
						pMP->ComputeDistinctiveDescriptors();
					}
					else // this can only happen for new stereo points inserted by the Tracking
					{
						m_recent_mps.push_back(pMP);
					}
				}
			}
		}

		// Update links in the Covisibility Graph
		m_cur_kf->UpdateConnections();

		// Insert Keyframe in Map
		m_map->AddKeyFrame(m_cur_kf);

	}

	void f_local_mapper::cull_mps()
	{
		// Check Recent Added MapPoints
		list<MapPoint*>::iterator lit = m_recent_mps.begin();
		const unsigned long int nCurrentKFid = m_cur_kf->mnId;

		int nThObs = 2;

		const int cnThObs = nThObs;

		while (lit != m_recent_mps.end())
		{
			MapPoint* pMP = *lit;
			if (pMP->isBad())
			{
				lit = m_recent_mps.erase(lit);
			}
			else if (pMP->GetFoundRatio()<0.25f)
			{
				pMP->SetBadFlag();
				lit = m_recent_mps.erase(lit);
			}
			else if (((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= 2 && pMP->Observations() <= cnThObs)
			{
				pMP->SetBadFlag();
				lit = m_recent_mps.erase(lit);
			}
			else if (((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= 3)
				lit = m_recent_mps.erase(lit);
			else
				lit++;
		}
	}

	void f_local_mapper::cull_kfs()
	{
		// Check redundant keyframes (only local keyframes)
		// A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
		// in at least other 3 keyframes (in the same or finer scale)
		// We only consider close stereo points
		vector<KeyFrame*> vpLocalKeyFrames = m_cur_kf->GetVectorCovisibleKeyFrames();

		for (vector<KeyFrame*>::iterator vit = vpLocalKeyFrames.begin(), vend = vpLocalKeyFrames.end(); vit != vend; vit++)
		{
			KeyFrame* pKF = *vit;
			if (pKF->mnId == 0)
				continue;
			const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

			int /*nObs = 2;
				if(mbMonocular)*/
				nObs = 3;
			const int thObs = nObs;
			int nRedundantObservations = 0;
			int nMPs = 0;
			for (size_t i = 0, iend = vpMapPoints.size(); i<iend; i++)
			{
				MapPoint* pMP = vpMapPoints[i];
				if (pMP)
				{
					if (!pMP->isBad())
					{

						nMPs++;
						if (pMP->Observations()>thObs)
						{
							const int &scaleLevel = pKF->mvKeysUn[i].octave;
							const map<KeyFrame*, size_t> observations = pMP->GetObservations();
							int nObs = 0;
							for (map<KeyFrame*, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
							{
								KeyFrame* pKFi = mit->first;
								if (pKFi == pKF)
									continue;
								const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

								if (scaleLeveli <= scaleLevel + 1)
								{
									nObs++;
									if (nObs >= thObs)
										break;
								}
							}
							if (nObs >= thObs)
							{
								nRedundantObservations++;
							}
						}
					}
				}
			}

			if (nRedundantObservations>0.9*nMPs)
				pKF->SetBadFlag();
		}
	}

	void f_local_mapper::create_new_mps()
	{
		// Retrieve neighbor keyframes in covisibility graph
		int nn = 20;
		const vector<KeyFrame*> vpNeighKFs = m_cur_kf->GetBestCovisibilityKeyFrames(nn);

		ORBmatcher matcher(0.6, false);

		cv::Mat Rcw1 = m_cur_kf->GetRotation();
		cv::Mat Rwc1 = Rcw1.t();
		cv::Mat tcw1 = m_cur_kf->GetTranslation();
		cv::Mat Tcw1(3, 4, CV_32F);
		Rcw1.copyTo(Tcw1.colRange(0, 3));
		tcw1.copyTo(Tcw1.col(3));
		cv::Mat Ow1 = m_cur_kf->GetCameraCenter();

		const float &fx1 = m_cur_kf->fx;
		const float &fy1 = m_cur_kf->fy;
		const float &cx1 = m_cur_kf->cx;
		const float &cy1 = m_cur_kf->cy;
		const float &invfx1 = m_cur_kf->invfx;
		const float &invfy1 = m_cur_kf->invfy;

		const float ratioFactor = 1.5f*m_cur_kf->mfScaleFactor;

		int nnew = 0;

		// Search matches with epipolar restriction and triangulate
		for (size_t i = 0; i<vpNeighKFs.size(); i++)
		{
			if (i>0 && !m_kf_tracker->empty())
				return;

			KeyFrame* pKF2 = vpNeighKFs[i];

			// Check first that baseline is not too short
			cv::Mat Ow2 = pKF2->GetCameraCenter();
			cv::Mat vBaseline = Ow2 - Ow1;
			const float baseline = cv::norm(vBaseline);

			const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
			const float ratioBaselineDepth = baseline / medianDepthKF2;

			if (ratioBaselineDepth < 0.01)
				continue;

			// Compute Fundamental Matrix
			cv::Mat F12 = compute_F12(m_cur_kf, pKF2);

			// Search matches that fullfil epipolar constraint
			vector<pair<size_t, size_t> > vMatchedIndices;
			matcher.SearchForTriangulation(m_cur_kf, pKF2, F12, vMatchedIndices, false);

			cv::Mat Rcw2 = pKF2->GetRotation();
			cv::Mat Rwc2 = Rcw2.t();
			cv::Mat tcw2 = pKF2->GetTranslation();
			cv::Mat Tcw2(3, 4, CV_32F);
			Rcw2.copyTo(Tcw2.colRange(0, 3));
			tcw2.copyTo(Tcw2.col(3));

			const float &fx2 = pKF2->fx;
			const float &fy2 = pKF2->fy;
			const float &cx2 = pKF2->cx;
			const float &cy2 = pKF2->cy;
			const float &invfx2 = pKF2->invfx;
			const float &invfy2 = pKF2->invfy;

			// Triangulate each match
			const int nmatches = vMatchedIndices.size();
			for (int ikp = 0; ikp<nmatches; ikp++)
			{
				const int &idx1 = vMatchedIndices[ikp].first;
				const int &idx2 = vMatchedIndices[ikp].second;

				const cv::KeyPoint &kp1 = m_cur_kf->mvKeysUn[idx1];
				const float kp1_ur = m_cur_kf->mvuRight[idx1];
				bool bStereo1 = kp1_ur >= 0;

				const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
				const float kp2_ur = pKF2->mvuRight[idx2];
				bool bStereo2 = kp2_ur >= 0;

				// Check parallax between rays
				cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.pt.x - cx1)*invfx1, (kp1.pt.y - cy1)*invfy1, 1.0);
				cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.pt.x - cx2)*invfx2, (kp2.pt.y - cy2)*invfy2, 1.0);

				cv::Mat ray1 = Rwc1*xn1;
				cv::Mat ray2 = Rwc2*xn2;
				const float cosParallaxRays = ray1.dot(ray2) / (cv::norm(ray1)*cv::norm(ray2));

				float cosParallaxStereo = cosParallaxRays + 1;
				float cosParallaxStereo1 = cosParallaxStereo;
				float cosParallaxStereo2 = cosParallaxStereo;

				if (bStereo1)
					cosParallaxStereo1 = cos(2 * atan2(m_cur_kf->mb / 2, m_cur_kf->mvDepth[idx1]));
				else if (bStereo2)
					cosParallaxStereo2 = cos(2 * atan2(pKF2->mb / 2, pKF2->mvDepth[idx2]));

				cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

				cv::Mat x3D;
				if (cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
				{
					// Linear Triangulation Method
					cv::Mat A(4, 4, CV_32F);
					A.row(0) = xn1.at<float>(0)*Tcw1.row(2) - Tcw1.row(0);
					A.row(1) = xn1.at<float>(1)*Tcw1.row(2) - Tcw1.row(1);
					A.row(2) = xn2.at<float>(0)*Tcw2.row(2) - Tcw2.row(0);
					A.row(3) = xn2.at<float>(1)*Tcw2.row(2) - Tcw2.row(1);

					cv::Mat w, u, vt;
					cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

					x3D = vt.row(3).t();

					if (x3D.at<float>(3) == 0)
						continue;

					// Euclidean coordinates
					x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);

				}
				else if (bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
				{
					x3D = m_cur_kf->UnprojectStereo(idx1);
				}
				else if (bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
				{
					x3D = pKF2->UnprojectStereo(idx2);
				}
				else
					continue; //No stereo and very low parallax

				cv::Mat x3Dt = x3D.t();

				//Check triangulation in front of cameras
				float z1 = Rcw1.row(2).dot(x3Dt) + tcw1.at<float>(2);
				if (z1 <= 0)
					continue;

				float z2 = Rcw2.row(2).dot(x3Dt) + tcw2.at<float>(2);
				if (z2 <= 0)
					continue;

				//Check reprojection error in first keyframe
				const float &sigmaSquare1 = m_cur_kf->mvLevelSigma2[kp1.octave];
				const float x1 = Rcw1.row(0).dot(x3Dt) + tcw1.at<float>(0);
				const float y1 = Rcw1.row(1).dot(x3Dt) + tcw1.at<float>(1);
				const float invz1 = 1.0 / z1;

				if (!bStereo1)
				{
					float u1 = fx1*x1*invz1 + cx1;
					float v1 = fy1*y1*invz1 + cy1;
					float errX1 = u1 - kp1.pt.x;
					float errY1 = v1 - kp1.pt.y;
					if ((errX1*errX1 + errY1*errY1)>5.991*sigmaSquare1)
						continue;
				}
				else
				{
					float u1 = fx1*x1*invz1 + cx1;
					float u1_r = u1 - m_cur_kf->mbf*invz1;
					float v1 = fy1*y1*invz1 + cy1;
					float errX1 = u1 - kp1.pt.x;
					float errY1 = v1 - kp1.pt.y;
					float errX1_r = u1_r - kp1_ur;
					if ((errX1*errX1 + errY1*errY1 + errX1_r*errX1_r)>7.8*sigmaSquare1)
						continue;
				}

				//Check reprojection error in second keyframe
				const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
				const float x2 = Rcw2.row(0).dot(x3Dt) + tcw2.at<float>(0);
				const float y2 = Rcw2.row(1).dot(x3Dt) + tcw2.at<float>(1);
				const float invz2 = 1.0 / z2;
				if (!bStereo2)
				{
					float u2 = fx2*x2*invz2 + cx2;
					float v2 = fy2*y2*invz2 + cy2;
					float errX2 = u2 - kp2.pt.x;
					float errY2 = v2 - kp2.pt.y;
					if ((errX2*errX2 + errY2*errY2)>5.991*sigmaSquare2)
						continue;
				}
				else
				{
					float u2 = fx2*x2*invz2 + cx2;
					float u2_r = u2 - m_cur_kf->mbf*invz2;
					float v2 = fy2*y2*invz2 + cy2;
					float errX2 = u2 - kp2.pt.x;
					float errY2 = v2 - kp2.pt.y;
					float errX2_r = u2_r - kp2_ur;
					if ((errX2*errX2 + errY2*errY2 + errX2_r*errX2_r)>7.8*sigmaSquare2)
						continue;
				}

				//Check scale consistency
				cv::Mat normal1 = x3D - Ow1;
				float dist1 = cv::norm(normal1);

				cv::Mat normal2 = x3D - Ow2;
				float dist2 = cv::norm(normal2);

				if (dist1 == 0 || dist2 == 0)
					continue;

				const float ratioDist = dist2 / dist1;
				const float ratioOctave = m_cur_kf->mvScaleFactors[kp1.octave] / pKF2->mvScaleFactors[kp2.octave];

				/*if(fabs(ratioDist-ratioOctave)>ratioFactor)
				continue;*/
				if (ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
					continue;

				// Triangulation is succesfull
				MapPoint* pMP = new MapPoint(x3D, m_cur_kf, m_map);

				pMP->AddObservation(m_cur_kf, idx1);
				pMP->AddObservation(pKF2, idx2);

				m_cur_kf->AddMapPoint(pMP, idx1);
				pKF2->AddMapPoint(pMP, idx2);

				pMP->ComputeDistinctiveDescriptors();

				pMP->UpdateNormalAndDepth();

				m_map->AddMapPoint(pMP);
				m_recent_mps.push_back(pMP);

				nnew++;
			}
		}

	}

	void f_local_mapper::search_neighbours()
	{
		// Retrieve neighbor keyframes
		int nn = 20;

		const vector<KeyFrame*> vpNeighKFs = m_cur_kf->GetBestCovisibilityKeyFrames(nn);
		vector<KeyFrame*> vpTargetKFs;
		for (vector<KeyFrame*>::const_iterator vit = vpNeighKFs.begin(), vend = vpNeighKFs.end(); vit != vend; vit++)
		{
			KeyFrame* pKFi = *vit;
			if (pKFi->isBad() || pKFi->mnFuseTargetForKF == m_cur_kf->mnId)
				continue;
			vpTargetKFs.push_back(pKFi);
			pKFi->mnFuseTargetForKF = m_cur_kf->mnId;

			// Extend to some second neighbors
			const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
			for (vector<KeyFrame*>::const_iterator vit2 = vpSecondNeighKFs.begin(), vend2 = vpSecondNeighKFs.end(); vit2 != vend2; vit2++)
			{
				KeyFrame* pKFi2 = *vit2;
				if (pKFi2->isBad() || pKFi2->mnFuseTargetForKF == m_cur_kf->mnId || pKFi2->mnId == m_cur_kf->mnId)
					continue;
				vpTargetKFs.push_back(pKFi2);
			}
		}


		// Search matches by projection from current KF in target KFs
		ORBmatcher matcher;
		vector<MapPoint*> vpMapPointMatches = m_cur_kf->GetMapPointMatches();
		for (vector<KeyFrame*>::iterator vit = vpTargetKFs.begin(), vend = vpTargetKFs.end(); vit != vend; vit++)
		{
			KeyFrame* pKFi = *vit;

			matcher.Fuse(pKFi, vpMapPointMatches);
		}

		// Search matches by projection from target KFs in current KF
		vector<MapPoint*> vpFuseCandidates;
		vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

		for (vector<KeyFrame*>::iterator vitKF = vpTargetKFs.begin(), vendKF = vpTargetKFs.end(); vitKF != vendKF; vitKF++)
		{
			KeyFrame* pKFi = *vitKF;

			vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

			for (vector<MapPoint*>::iterator vitMP = vpMapPointsKFi.begin(), vendMP = vpMapPointsKFi.end(); vitMP != vendMP; vitMP++)
			{
				MapPoint* pMP = *vitMP;
				if (!pMP)
					continue;
				if (pMP->isBad() || pMP->mnFuseCandidateForKF == m_cur_kf->mnId)
					continue;
				pMP->mnFuseCandidateForKF = m_cur_kf->mnId;
				vpFuseCandidates.push_back(pMP);
			}
		}

		matcher.Fuse(m_cur_kf, vpFuseCandidates);


		// Update points
		vpMapPointMatches = m_cur_kf->GetMapPointMatches();
		for (size_t i = 0, iend = vpMapPointMatches.size(); i<iend; i++)
		{
			MapPoint* pMP = vpMapPointMatches[i];
			if (pMP)
			{
				if (!pMP->isBad())
				{
					pMP->ComputeDistinctiveDescriptors();
					pMP->UpdateNormalAndDepth();
				}
			}
		}

		// Update connections in covisibility graph
		m_cur_kf->UpdateConnections();
	}

	Mat f_local_mapper::convSkewSymmetricMatrix(const Mat & v)
	{
		return (cv::Mat_<float>(3, 3) << 0, -v.at<float>(2), v.at<float>(1),
			v.at<float>(2), 0, -v.at<float>(0),
			-v.at<float>(1), v.at<float>(0), 0);
	}

	Mat f_local_mapper::compute_F12(KeyFrame * & pKF1, KeyFrame * & pKF2)
	{ 
		cv::Mat R1w = pKF1->GetRotation();
		cv::Mat t1w = pKF1->GetTranslation();
		cv::Mat R2w = pKF2->GetRotation();
		cv::Mat t2w = pKF2->GetTranslation();

		cv::Mat R12 = R1w*R2w.t();
		cv::Mat t12 = -R1w*R2w.t()*t2w + t1w;

		cv::Mat t12x = convSkewSymmetricMatrix(t12);

		const cv::Mat &K1 = pKF1->mK;
		const cv::Mat &K2 = pKF2->mK;


		return K1.t().inv()*t12x*R12*K2.inv();
	}

	/////////////////////////////////////////////////// loop closer
	f_loop_closer::f_loop_closer(const char * name) :f_base(name),
		m_sys(NULL), m_kf_mapper(NULL), m_map(NULL), m_kfdb(NULL), m_cur_kf(NULL),
		m_th_covisibility_consistency(3.f), m_fix_scale(true)
	{
		register_fpar("ch_sys", (ch_base**)&m_sys, typeid(ch_sys).name(), "System channel.");
		register_fpar("ch_kf_mapper", (ch_base**)&m_kf_mapper, typeid(ch_keyframe).name(), "Key frame channel from mapper. ");
		register_fpar("ch_map", (ch_base**)&m_map, typeid(ch_map).name(), "Map channel");
		register_fpar("ch_kfdb", (ch_base**)&m_kfdb, typeid(ch_keyframeDB).name(), "Key frame database channel.");

		register_fpar("thcc", &m_th_covisibility_consistency, "Threashold for covisibility consistency.");
		register_fpar("fix_scale", &m_fix_scale, "Fix scale flag.");
	}
	
	f_loop_closer::~f_loop_closer()
	{
	}

	bool f_loop_closer::init_run()
	{
		if (!m_sys){
			cerr << "Channel sys not found in f_loop_closer::init_run()" << endl;
			return false;
		}

		if (!m_kf_mapper){
			cerr << "Channel kf_mapper not found in f_loop_closer::init_run()" << endl;
			return false;
		}

		return true;
	}

	void f_loop_closer::destroy_run()
	{
	}

	bool f_loop_closer::proc()
	{
		// Check if there are keyframes in the queue
		if (!m_kf_mapper->empty())
		{
			// Detect loop candidates and check covisibility consistency
			if (detect_loop())
			{
				// Compute similarity transformation [sR|t]
				// In the stereo/RGBD case s=1
				if (compute_sim3())
				{
					// Perform loop fusion and pose graph optimization
					correct_loop();
				}
			}
		}

		if (m_sys->is_rst_loop_closer()){
			m_last_loop_kf_id = 0;
			m_kf_mapper->clear();
			m_sys->rst_done_loop_closer();
		}

		return true;
	}

	bool f_loop_closer::detect_loop()
	{
		m_cur_kf = m_kf_mapper->pop();

		//If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
		if (m_cur_kf->mnId<m_last_loop_kf_id + 10)
		{
			m_kfdb->add(m_cur_kf);
			m_cur_kf->SetErase();
			return false;
		}

		// Compute reference BoW similarity score
		// This is the lowest score to a connected keyframe in the covisibility graph
		// We will impose loop candidates to have a higher similarity than this
		const vector<KeyFrame*> vpConnectedKeyFrames = m_cur_kf->GetVectorCovisibleKeyFrames();
		const DBoW2::BowVector &CurrentBowVec = m_cur_kf->mBowVec;
		float minScore = 1;
		for (size_t i = 0; i<vpConnectedKeyFrames.size(); i++)
		{
			KeyFrame* pKF = vpConnectedKeyFrames[i];
			if (pKF->isBad())
				continue;
			const DBoW2::BowVector &BowVec = pKF->mBowVec;

			ORBVocabulary * pvoc = m_kfdb->get_orb_voc();
			float score = pvoc->score(CurrentBowVec, BowVec);

			if (score<minScore)
				minScore = score;
		}

		// Query the database imposing the minimum score
		vector<KeyFrame*> vpCandidateKFs = m_kfdb->DetectLoopCandidates(m_cur_kf, minScore);

		// If there are no loop candidates, just add new keyframe and return false
		if (vpCandidateKFs.empty())
		{
			m_kfdb->add(m_cur_kf);
			m_consistent_groups.clear();
			m_cur_kf->SetErase();
			return false;
		}

		// For each loop candidate check consistency with previous loop candidates
		// Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
		// A group is consistent with a previous group if they share at least a keyframe
		// We must detect a consistent loop in several consecutive keyframes to accept it
		m_enough_consistent_candidates.clear();

		vector<ConsistentGroup> vCurrentConsistentGroups;
		vector<bool> vbConsistentGroup(m_consistent_groups.size(), false);
		for (size_t i = 0, iend = vpCandidateKFs.size(); i<iend; i++)
		{
			KeyFrame* pCandidateKF = vpCandidateKFs[i];

			set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
			spCandidateGroup.insert(pCandidateKF);

			bool bEnoughConsistent = false;
			bool bConsistentForSomeGroup = false;
			for (size_t iG = 0, iendG = m_consistent_groups.size(); iG<iendG; iG++)
			{
				set<KeyFrame*> sPreviousGroup = m_consistent_groups[iG].first;

				bool bConsistent = false;
				for (set<KeyFrame*>::iterator sit = spCandidateGroup.begin(), send = spCandidateGroup.end(); sit != send; sit++)
				{
					if (sPreviousGroup.count(*sit))
					{
						bConsistent = true;
						bConsistentForSomeGroup = true;
						break;
					}
				}

				if (bConsistent)
				{
					int nPreviousConsistency = m_consistent_groups[iG].second;
					int nCurrentConsistency = nPreviousConsistency + 1;
					if (!vbConsistentGroup[iG])
					{
						ConsistentGroup cg = make_pair(spCandidateGroup, nCurrentConsistency);
						vCurrentConsistentGroups.push_back(cg);
						vbConsistentGroup[iG] = true; //this avoid to include the same group more than once
					}
					if (nCurrentConsistency >= m_th_covisibility_consistency && !bEnoughConsistent)
					{
						m_enough_consistent_candidates.push_back(pCandidateKF);
						bEnoughConsistent = true; //this avoid to insert the same candidate more than once
					}
				}
			}

			// If the group is not consistent with any previous group insert with consistency counter set to zero
			if (!bConsistentForSomeGroup)
			{
				ConsistentGroup cg = make_pair(spCandidateGroup, 0);
				vCurrentConsistentGroups.push_back(cg);
			}
		}

		// Update Covisibility Consistent Groups
		m_consistent_groups = vCurrentConsistentGroups;


		// Add Current Keyframe to database
		m_kfdb->add(m_cur_kf);

		if (m_enough_consistent_candidates.empty())
		{
			m_cur_kf->SetErase();
			return false;
		}
		else
		{
			return true;
		}

		m_cur_kf->SetErase();
		return false;
	}

	bool f_loop_closer::compute_sim3()
	{
		// For each consistent loop candidate we try to compute a Sim3

		const int nInitialCandidates = m_enough_consistent_candidates.size();

		// We compute first ORB matches for each candidate
		// If enough matches are found, we setup a Sim3Solver
		ORBmatcher matcher(0.75, true);

		vector<Sim3Solver*> vpSim3Solvers;
		vpSim3Solvers.resize(nInitialCandidates);

		vector<vector<MapPoint*> > vvpMapPointMatches;
		vvpMapPointMatches.resize(nInitialCandidates);

		vector<bool> vbDiscarded;
		vbDiscarded.resize(nInitialCandidates);

		int nCandidates = 0; //candidates with enough matches

		for (int i = 0; i<nInitialCandidates; i++)
		{
			KeyFrame* pKF = m_enough_consistent_candidates[i];

			// avoid that local mapping erase it while it is being processed in this thread
			pKF->SetNotErase();

			if (pKF->isBad())
			{
				vbDiscarded[i] = true;
				continue;
			}

			int nmatches = matcher.SearchByBoW(m_cur_kf, pKF, vvpMapPointMatches[i]);

			if (nmatches<20)
			{
				vbDiscarded[i] = true;
				continue;
			}
			else
			{
				Sim3Solver* pSolver = new Sim3Solver(m_cur_kf, pKF, vvpMapPointMatches[i], m_fix_scale);
				pSolver->SetRansacParameters(0.99, 20, 300);
				vpSim3Solvers[i] = pSolver;
			}

			nCandidates++;
		}

		bool bMatch = false;

		// Perform alternatively RANSAC iterations for each candidate
		// until one is succesful or all fail
		while (nCandidates>0 && !bMatch)
		{
			for (int i = 0; i<nInitialCandidates; i++)
			{
				if (vbDiscarded[i])
					continue;

				KeyFrame* pKF = m_enough_consistent_candidates[i];

				// Perform 5 Ransac Iterations
				vector<bool> vbInliers;
				int nInliers;
				bool bNoMore;

				Sim3Solver* pSolver = vpSim3Solvers[i];
				cv::Mat Scm = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

				// If Ransac reachs max. iterations discard keyframe
				if (bNoMore)
				{
					vbDiscarded[i] = true;
					nCandidates--;
				}

				// If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
				if (!Scm.empty())
				{
					vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
					for (size_t j = 0, jend = vbInliers.size(); j<jend; j++)
					{
						if (vbInliers[j])
							vpMapPointMatches[j] = vvpMapPointMatches[i][j];
					}

					cv::Mat R = pSolver->GetEstimatedRotation();
					cv::Mat t = pSolver->GetEstimatedTranslation();
					const float s = pSolver->GetEstimatedScale();
					matcher.SearchBySim3(m_cur_kf, pKF, vpMapPointMatches, s, R, t, 7.5);

					g2o::Sim3 gScm(Converter::toMatrix3d(R), Converter::toVector3d(t), s);
					const int nInliers = Optimizer::OptimizeSim3(m_cur_kf, pKF, vpMapPointMatches, gScm, 10, m_fix_scale);

					// If optimization is succesful stop ransacs and continue
					if (nInliers >= 20)
					{
						bMatch = true;
						m_matched_kf = pKF;
						g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()), Converter::toVector3d(pKF->GetTranslation()), 1.0);
						m_Scw_g2o = gScm*gSmw;
						m_Scw = Converter::toCvMat(m_Scw_g2o);

						m_cur_matched_mps = vpMapPointMatches;
						break;
					}
				}
			}
		}

		if (!bMatch)
		{
			for (int i = 0; i<nInitialCandidates; i++)
				m_enough_consistent_candidates[i]->SetErase();
			m_cur_kf->SetErase();
			return false;
		}

		// Retrieve MapPoints seen in Loop Keyframe and neighbors
		vector<KeyFrame*> vpLoopConnectedKFs = m_matched_kf->GetVectorCovisibleKeyFrames();
		vpLoopConnectedKFs.push_back(m_matched_kf);
		m_loop_mps.clear();
		for (vector<KeyFrame*>::iterator vit = vpLoopConnectedKFs.begin(); vit != vpLoopConnectedKFs.end(); vit++)
		{
			KeyFrame* pKF = *vit;
			vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
			for (size_t i = 0, iend = vpMapPoints.size(); i<iend; i++)
			{
				MapPoint* pMP = vpMapPoints[i];
				if (pMP)
				{
					if (!pMP->isBad() && pMP->mnLoopPointForKF != m_cur_kf->mnId)
					{
						m_loop_mps.push_back(pMP);
						pMP->mnLoopPointForKF = m_cur_kf->mnId;
					}
				}
			}
		}

		// Find more matches projecting with the computed Sim3
		matcher.SearchByProjection(m_cur_kf, m_Scw, m_loop_mps, m_cur_matched_mps, 10);

		// If enough matches accept Loop
		int nTotalMatches = 0;
		for (size_t i = 0; i<m_cur_matched_mps.size(); i++)
		{
			if (m_cur_matched_mps[i])
				nTotalMatches++;
		}

		if (nTotalMatches >= 40)
		{
			for (int i = 0; i<nInitialCandidates; i++)
			if (m_enough_consistent_candidates[i] != m_matched_kf)
				m_enough_consistent_candidates[i]->SetErase();
			return true;
		}
		else
		{
			for (int i = 0; i<nInitialCandidates; i++)
				m_enough_consistent_candidates[i]->SetErase();
			m_cur_kf->SetErase();
			return false;
		}

	}

	void f_loop_closer::correct_loop()
	{
		cout << "Loop detected!" << endl;

		// Send a stop signal to Local Mapping
		// Avoid new keyframes are inserted while correcting the loop
		m_sys->req_stop_mapper();
		
		// If a Global Bundle Adjustment is running, abort it
		if (m_sys->is_running_gba())
		{
			m_sys->set_stop_gba(true);

			while (!m_sys->is_finish_gba()){
#ifdef _WIN32
				Sleep(5);
#else
				usleep(5000);
#endif
			}

			m_thread_ba->join();
			delete m_thread_ba;
		}

		// Wait until Local Mapping has effectively stopped
		while (!m_sys->is_stopped_mapper())
		{
#ifdef _WIN32
			Sleep(1);
#else
			usleep(1000);
#endif
		}

		// Ensure current keyframe is updated
		m_cur_kf->UpdateConnections();

		// Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
		m_cur_connected_kfs = m_cur_kf->GetVectorCovisibleKeyFrames();
		m_cur_connected_kfs.push_back(m_cur_kf);

		KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
		CorrectedSim3[m_cur_kf] = m_Scw_g2o;
		cv::Mat Twc = m_cur_kf->GetPoseInverse();

		{
			// Get Map Mutex
			unique_lock<mutex> lock(m_map->mMutexMapUpdate);

			for (vector<KeyFrame*>::iterator vit = m_cur_connected_kfs.begin(), vend = m_cur_connected_kfs.end(); vit != vend; vit++)
			{
				KeyFrame* pKFi = *vit;

				cv::Mat Tiw = pKFi->GetPose();

				if (pKFi != m_cur_kf)
				{
					cv::Mat Tic = Tiw*Twc;
					cv::Mat Ric = Tic.rowRange(0, 3).colRange(0, 3);
					cv::Mat tic = Tic.rowRange(0, 3).col(3);
					g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric), Converter::toVector3d(tic), 1.0);
					g2o::Sim3 g2oCorrectedSiw = g2oSic*m_Scw_g2o;
					//Pose corrected with the Sim3 of the loop closure
					CorrectedSim3[pKFi] = g2oCorrectedSiw;
				}

				cv::Mat Riw = Tiw.rowRange(0, 3).colRange(0, 3);
				cv::Mat tiw = Tiw.rowRange(0, 3).col(3);
				g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw), Converter::toVector3d(tiw), 1.0);
				//Pose without correction
				NonCorrectedSim3[pKFi] = g2oSiw;
			}

			// Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
			for (KeyFrameAndPose::iterator mit = CorrectedSim3.begin(), mend = CorrectedSim3.end(); mit != mend; mit++)
			{
				KeyFrame* pKFi = mit->first;
				g2o::Sim3 g2oCorrectedSiw = mit->second;
				g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

				g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

				vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
				for (size_t iMP = 0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
				{
					MapPoint* pMPi = vpMPsi[iMP];
					if (!pMPi)
						continue;
					if (pMPi->isBad())
						continue;
					if (pMPi->mnCorrectedByKF == m_cur_kf->mnId)
						continue;

					// Project with non-corrected pose and project back with corrected pose
					cv::Mat P3Dw = pMPi->GetWorldPos();
					Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
					Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

					cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
					pMPi->SetWorldPos(cvCorrectedP3Dw);
					pMPi->mnCorrectedByKF = m_cur_kf->mnId;
					pMPi->mnCorrectedReference = pKFi->mnId;
					pMPi->UpdateNormalAndDepth();
				}

				// Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
				Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
				Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
				double s = g2oCorrectedSiw.scale();

				eigt *= (1. / s); //[R t/s;0 1]

				cv::Mat correctedTiw = Converter::toCvSE3(eigR, eigt);

				pKFi->SetPose(correctedTiw);

				// Make sure connections are updated
				pKFi->UpdateConnections();
			}

			// Start Loop Fusion
			// Update matched map points and replace if duplicated
			for (size_t i = 0; i<m_cur_matched_mps.size(); i++)
			{
				if (m_cur_matched_mps[i])
				{
					MapPoint* pLoopMP = m_cur_matched_mps[i];
					MapPoint* pCurMP = m_cur_kf->GetMapPoint(i);
					if (pCurMP)
						pCurMP->Replace(pLoopMP);
					else
					{
						m_cur_kf->AddMapPoint(pLoopMP, i);
						pLoopMP->AddObservation(m_cur_kf, i);
						pLoopMP->ComputeDistinctiveDescriptors();
					}
				}
			}

		}

		// Project MapPoints observed in the neighborhood of the loop keyframe
		// into the current keyframe and neighbors using corrected poses.
		// Fuse duplications.
		search_and_fuse(CorrectedSim3);

		// After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
		map<KeyFrame*, set<KeyFrame*> > LoopConnections;

		for (vector<KeyFrame*>::iterator vit = m_cur_connected_kfs.begin(), vend = m_cur_connected_kfs.end(); vit != vend; vit++)
		{
			KeyFrame* pKFi = *vit;
			vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

			// Update connections. Detect new links.
			pKFi->UpdateConnections();
			LoopConnections[pKFi] = pKFi->GetConnectedKeyFrames();
			for (vector<KeyFrame*>::iterator vit_prev = vpPreviousNeighbors.begin(), vend_prev = vpPreviousNeighbors.end(); vit_prev != vend_prev; vit_prev++)
			{
				LoopConnections[pKFi].erase(*vit_prev);
			}
			for (vector<KeyFrame*>::iterator vit2 = m_cur_connected_kfs.begin(), vend2 = m_cur_connected_kfs.end(); vit2 != vend2; vit2++)
			{
				LoopConnections[pKFi].erase(*vit2);
			}
		}

		// Optimize graph
		Optimizer::OptimizeEssentialGraph(m_map, m_matched_kf, m_cur_kf, NonCorrectedSim3, CorrectedSim3, LoopConnections, m_fix_scale);

		// Add loop edge
		m_matched_kf->AddLoopEdge(m_cur_kf);
		m_cur_kf->AddLoopEdge(m_matched_kf);

		// Launch a new thread to perform Global Bundle Adjustment
		m_sys->set_running_gba(true);
		m_sys->set_stop_gba(false);
		m_sys->set_finish_gba(false);
		m_thread_ba = new thread(&f_loop_closer::run_gba, this, m_cur_kf->mnId);

		// Loop closed. Release Local Mapping.
		m_sys->release_mapper();

		cout << "Loop Closed!" << endl;
		m_last_loop_kf_id = m_cur_kf->mnId;
	}

	void f_loop_closer::search_and_fuse(const KeyFrameAndPose & corrected_pose_map)
	{
		ORBmatcher matcher(0.8);

		for (KeyFrameAndPose::const_iterator mit = corrected_pose_map.begin(), mend = corrected_pose_map.end(); mit != mend; mit++)
		{
			KeyFrame* pKF = mit->first;

			g2o::Sim3 g2oScw = mit->second;
			cv::Mat cvScw = Converter::toCvMat(g2oScw);

			vector<MapPoint*> vpReplacePoints(m_loop_mps.size(), static_cast<MapPoint*>(NULL));
			matcher.Fuse(pKF, cvScw, m_loop_mps, 4, vpReplacePoints);

			// Get Map Mutex
			unique_lock<mutex> lock(m_map->mMutexMapUpdate);
			const int nLP = m_loop_mps.size();
			for (int i = 0; i<nLP; i++)
			{
				MapPoint* pRep = vpReplacePoints[i];
				if (pRep)
				{
					pRep->Replace(m_loop_mps[i]);
				}
			}
		}
	}

	void f_loop_closer::run_gba(unsigned long loop_kf_id)
	{
		cout << "Starting Global Bundle Adjustment" << endl;

		Optimizer::GlobalBundleAdjustemnt(m_map, 20, m_sys->export_stop_gba(), loop_kf_id, false);

		// Update all MapPoints and KeyFrames
		// Local Mapping was active during BA, that means that there might be new keyframes
		// not included in the Global BA and they are not consistent with the updated map.
		// We need to propagate the correction through the spanning tree
		{

			if (!m_sys->is_stop_gba())
			{
				cout << "Global Bundle Adjustment finished" << endl;
				cout << "Updating map ..." << endl;
				m_sys->req_stop_mapper();
				// Wait until Local Mapping has effectively stopped

				while (!m_sys->is_stopped_mapper() && !m_sys->is_finished_mapper())
				{
#ifdef _WIN32
					Sleep(1);
#else			
					usleep(1000);
#endif
				}

				// Get Map Mutex
				unique_lock<mutex> lock(m_map->mMutexMapUpdate);

				// Correct keyframes starting at map first keyframe
				list<KeyFrame*> lpKFtoCheck(m_map->mvpKeyFrameOrigins.begin(), m_map->mvpKeyFrameOrigins.end());

				while (!lpKFtoCheck.empty())
				{
					KeyFrame* pKF = lpKFtoCheck.front();
					const set<KeyFrame*> sChilds = pKF->GetChilds();
					cv::Mat Twc = pKF->GetPoseInverse();
					for (set<KeyFrame*>::const_iterator sit = sChilds.begin(); sit != sChilds.end(); sit++)
					{
						KeyFrame* pChild = *sit;
						if (pChild->mnBAGlobalForKF != loop_kf_id)
						{
							cv::Mat Tchildc = pChild->GetPose()*Twc;
							pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
							pChild->mnBAGlobalForKF = loop_kf_id;

						}
						lpKFtoCheck.push_back(pChild);
					}

					pKF->mTcwBefGBA = pKF->GetPose();
					pKF->SetPose(pKF->mTcwGBA);
					lpKFtoCheck.pop_front();
				}

				// Correct MapPoints
				const vector<MapPoint*> vpMPs = m_map->GetAllMapPoints();

				for (size_t i = 0; i<vpMPs.size(); i++)
				{
					MapPoint* pMP = vpMPs[i];

					if (pMP->isBad())
						continue;

					if (pMP->mnBAGlobalForKF == loop_kf_id)
					{
						// If optimized by Global BA, just update
						pMP->SetWorldPos(pMP->mPosGBA);
					}
					else
					{
						// Update according to the correction of its reference keyframe
						KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

						if (pRefKF->mnBAGlobalForKF != loop_kf_id)
							continue;

						// Map to non-corrected camera
						cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0, 3).colRange(0, 3);
						cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0, 3).col(3);
						cv::Mat Xc = Rcw*pMP->GetWorldPos() + tcw;

						// Backproject using corrected camera
						cv::Mat Twc = pRefKF->GetPoseInverse();
						cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
						cv::Mat twc = Twc.rowRange(0, 3).col(3);

						pMP->SetWorldPos(Rwc*Xc + twc);
					}
				}

				m_sys->release_mapper();
				cout << "Map updated!" << endl;
			}

			m_sys->set_finish_gba(true);
			m_sys->set_running_gba(false);
		}
	}


	const char * f_viewer::m_str_vmode[UNDEF] = {
		"f", "m", "fm"
	};

	f_viewer::f_viewer(const char * name) : f_glfw_window(name), m_cam(NULL), m_sys(NULL),
		m_map(NULL), m_trj(NULL), m_frm(NULL),
		m_draw_kf(true), m_draw_g(true), m_vmode(FRAME)
	{
		register_fpar("ch_sys", (ch_base**)&m_sys, typeid(ch_sys).name(), "System channel.");
		register_fpar("ch_cam", (ch_base**)&m_cam, typeid(ch_image_ref).name(), "Camera image channel.");
		register_fpar("ch_map", (ch_base**)&m_map, typeid(ch_map).name(), "Map channel.");
		register_fpar("ch_trj", (ch_base**)&m_trj, typeid(ch_trj).name(), "Trajectory channel.");
		register_fpar("ch_frm", (ch_base**)&m_frm, typeid(ch_frm).name(), "Frame channel.");
		register_fpar("draw_kf", &m_draw_kf, "Flag drawing key frames.");
		register_fpar("draw_g", &m_draw_g, "Flag drawing graph.");

		register_fpar("sz_kf", &m_sz_kf, "Size of key frame.");
		register_fpar("lw_kf", &m_lw_kf, "Line widht of key frame");
		register_fpar("sz_pt", &m_sz_pt, "Size of point.");
		register_fpar("sz_cam", &m_sz_cam, "size of camera.");
		register_fpar("lw_cam", &m_lw_cam, "line width of camera.");


		register_fpar("fovy", &m_fovy, "Field of view");
		register_fpar("eyex", &m_eyex, "x point looking at");
		register_fpar("eyey", &m_eyey, "y point looking at");
		register_fpar("eyez", &m_eyez, "z point looking at");
		register_fpar("zmin", &m_zmin, "Minimum scene depth");
		register_fpar("zmax", &m_zmax, "Maximum scene depth");

		register_fpar("mode", (int*)&m_vmode, UNDEF, m_str_vmode, "View Mode");
	}

	f_viewer::~f_viewer()
	{
	}

	bool f_viewer::init_run()
	{

		if (!f_glfw_window::init_run())
			return false;

		// 3D Mouse handler requires depth testing to be enabled
		glEnable(GL_DEPTH_TEST);

		// Issue specific OpenGl we might need
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		m_ar = (double)m_sz_win.width / (double)m_sz_win.height;

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(m_fovy, m_ar, m_zmin, m_zmax);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(m_eyex, m_eyey, m_eyez, 0, 0, 0, 0, -1., 0);

		return true;
	}

	void f_viewer::destroy_run()
	{
		f_glfw_window::destroy_run();
	}

	bool f_viewer::proc()
	{
		glfwMakeContextCurrent(pwin());

		if (glfwWindowShouldClose(pwin()))
			return false;

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

		Mat imWithInfo;
		if (m_frm && m_map && m_vmode != MAP){
			draw_overlay_info(imWithInfo);
		}

		long long timg;
		if (imWithInfo.empty())
			return true;


		glRasterPos2i(-1, -1);
		if (m_vmode == FRAME){
			if (m_sz_win.width != imWithInfo.cols || m_sz_win.height != imWithInfo.rows){
				Mat tmp;
				resize(imWithInfo, tmp, m_sz_win);
				imWithInfo = tmp;
			}
			glDrawPixels(imWithInfo.cols, imWithInfo.rows, GL_RGB, GL_UNSIGNED_BYTE, imWithInfo.data);
		}
		else if (m_vmode == MAP_AND_FRAME){
			if (m_sz_win.width >> 2 != imWithInfo.cols || m_sz_win.height >> 2 != imWithInfo.rows){
				Mat tmp;
				resize(imWithInfo, tmp, Size(m_sz_win.width >> 2, m_sz_win.height >> 2));
				imWithInfo = tmp;
			}
			glDrawPixels(imWithInfo.cols, imWithInfo.rows, GL_RGB, GL_UNSIGNED_BYTE, imWithInfo.data);
		}


		if (m_vmode != FRAME){
			GLdouble Twc[16];
			load_cam_pose(Twc);
			draw_cam(Twc);

			draw_kfs();
			draw_mps();
		}

		glfwSwapBuffers(pwin());
		glfwPollEvents();

		return true;
	}

	void f_viewer::draw_overlay_info(Mat & imWithInfo)
	{
		cv::Mat im;
		vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
		vector<int> vMatches; // Initialization: correspondeces with reference keypoints
		vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
		vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
		e_tracking_state state; // Tracking state
		m_frm->get(im, vMatches, vIniKeys, vCurrentKeys, vbVO, vbMap, state);

		if (im.channels() < 3) //this should be always true
			cvtColor(im, im, CV_GRAY2BGR);

		//Draw
		int N = vCurrentKeys.size();

		if (state == NOT_INITIALIZED) //INITIALIZING
		{
			for (unsigned int i = 0; i < vMatches.size(); i++)
			{
				if (vMatches[i] >= 0)
				{
					cv::line(im, vIniKeys[i].pt, vCurrentKeys[vMatches[i]].pt,
						cv::Scalar(0, 255, 0));
				}
			}
		}
		else if (state == OK) //TRACKING
		{
			m_num_tracked = 0;
			m_num_tracked_vo = 0;
			const float r = 5;
			for (int i = 0; i < N; i++)
			{
				if (vbVO[i] || vbMap[i])
				{
					cv::Point2f pt1, pt2;
					pt1.x = vCurrentKeys[i].pt.x - r;
					pt1.y = vCurrentKeys[i].pt.y - r;
					pt2.x = vCurrentKeys[i].pt.x + r;
					pt2.y = vCurrentKeys[i].pt.y + r;

					// This is a match to a MapPoint in the map
					if (vbMap[i])
					{
						cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
						cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(0, 255, 0), -1);
						m_num_tracked++;
					}
					else // This is match to a "visual odometry" MapPoint created in the last frame
					{
						cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 0));
						cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(255, 0, 0), -1);
						m_num_tracked_vo++;
					}
				}
			}
		}


		stringstream s;
		if (state == NO_IMAGES_YET)
			s << " WAITING FOR IMAGES";
		else if (state == NOT_INITIALIZED)
			s << " TRYING TO INITIALIZE ";
		else if (state == OK)
		{
			s << "SLAM MODE |  ";
			int nKFs = m_map->KeyFramesInMap();
			int nMPs = m_map->MapPointsInMap();
			s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << m_num_tracked;
			if (m_num_tracked_vo > 0)
				s << ", + VO matches: " << m_num_tracked_vo;
		}
		else if (state == LOST)
		{
			s << " TRACK LOST. TRYING TO RELOCALIZE ";
		}
		else if (state == SYSTEM_NOT_READY)
		{
			s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
		}

		int baseline = 0;
		cv::Size textSize = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

		imWithInfo = cv::Mat(im.rows + textSize.height + 10, im.cols, im.type());
		im.copyTo(imWithInfo.rowRange(0, im.rows).colRange(0, im.cols));
		imWithInfo.rowRange(im.rows, imWithInfo.rows) = cv::Mat::zeros(textSize.height + 10, im.cols, im.type());
		cv::putText(imWithInfo, s.str(), cv::Point(5, imWithInfo.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);

	}

	void f_viewer::draw_mps()
	{
		const vector<MapPoint*> &vpMPs = m_map->GetAllMapPoints();
		const vector<MapPoint*> &vpRefMPs = m_map->GetReferenceMapPoints();

		set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

		if (vpMPs.empty())
			return;

		glPointSize(m_sz_pt);
		glBegin(GL_POINTS);
		glColor3f(0.0, 0.0, 0.0);

		for (size_t i = 0, iend = vpMPs.size(); i<iend; i++)
		{
			if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
				continue;
			cv::Mat pos = vpMPs[i]->GetWorldPos();
			glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
		}
		glEnd();

		glPointSize(m_sz_pt);
		glBegin(GL_POINTS);
		glColor3f(1.0, 0.0, 0.0);

		for (set<MapPoint*>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++)
		{
			if ((*sit)->isBad())
				continue;
			cv::Mat pos = (*sit)->GetWorldPos();
			glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));

		}

		glEnd();
	}

	void f_viewer::draw_kfs()
	{
		const float &w = m_sz_kf;
		const float h = w*0.75;
		const float z = w*0.6;

		const vector<KeyFrame*> vpKFs = m_map->GetAllKeyFrames();

		if (m_draw_kf)
		{
			for (size_t i = 0; i<vpKFs.size(); i++)
			{
				KeyFrame* pKF = vpKFs[i];
				cv::Mat Twc = pKF->GetPoseInverse().t();

				glPushMatrix();

				glMultMatrixf(Twc.ptr<GLfloat>(0));

				glLineWidth(m_lw_kf);
				glColor3f(0.0f, 0.0f, 1.0f);
				glBegin(GL_LINES);
				glVertex3f(0, 0, 0);
				glVertex3f(w, h, z);
				glVertex3f(0, 0, 0);
				glVertex3f(w, -h, z);
				glVertex3f(0, 0, 0);
				glVertex3f(-w, -h, z);
				glVertex3f(0, 0, 0);
				glVertex3f(-w, h, z);

				glVertex3f(w, h, z);
				glVertex3f(w, -h, z);

				glVertex3f(-w, h, z);
				glVertex3f(-w, -h, z);

				glVertex3f(-w, h, z);
				glVertex3f(w, h, z);

				glVertex3f(-w, -h, z);
				glVertex3f(w, -h, z);
				glEnd();

				glPopMatrix();
			}
		}

		if (m_draw_g)
		{
			glLineWidth(m_lw_g);
			glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
			glBegin(GL_LINES);

			for (size_t i = 0; i<vpKFs.size(); i++)
			{
				// Covisibility Graph
				const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
				cv::Mat Ow = vpKFs[i]->GetCameraCenter();
				if (!vCovKFs.empty())
				{
					for (vector<KeyFrame*>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end(); vit != vend; vit++)
					{
						if ((*vit)->mnId<vpKFs[i]->mnId)
							continue;
						cv::Mat Ow2 = (*vit)->GetCameraCenter();
						glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
						glVertex3f(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2));
					}
				}

				// Spanning tree
				KeyFrame* pParent = vpKFs[i]->GetParent();
				if (pParent)
				{
					cv::Mat Owp = pParent->GetCameraCenter();
					glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
					glVertex3f(Owp.at<float>(0), Owp.at<float>(1), Owp.at<float>(2));
				}

				// Loops
				set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
				for (set<KeyFrame*>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++)
				{
					if ((*sit)->mnId<vpKFs[i]->mnId)
						continue;
					cv::Mat Owl = (*sit)->GetCameraCenter();
					glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
					glVertex3f(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2));
				}
			}

			glEnd();
		}
	}

	void f_viewer::draw_cam(GLdouble * Twc)
	{
		const float &w = m_sz_cam;
		const float h = w*0.75;
		const float z = w*0.6;

		glPushMatrix();

#ifdef HAVE_GLES
		glMultMatrixf(Twc);
#else
		glMultMatrixd(Twc);
#endif

		glLineWidth(m_lw_cam);
		glColor3f(0.0f, 1.0f, 0.0f);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(w, h, z);
		glVertex3f(0, 0, 0);
		glVertex3f(w, -h, z);
		glVertex3f(0, 0, 0);
		glVertex3f(-w, -h, z);
		glVertex3f(0, 0, 0);
		glVertex3f(-w, h, z);

		glVertex3f(w, h, z);
		glVertex3f(w, -h, z);

		glVertex3f(-w, h, z);
		glVertex3f(-w, -h, z);

		glVertex3f(-w, h, z);
		glVertex3f(w, h, z);

		glVertex3f(-w, -h, z);
		glVertex3f(w, -h, z);
		glEnd();

		glPopMatrix();
	}

	void f_viewer::load_cam_pose(GLdouble * Twc)
	{
		Mat _Tcw = m_frm->get_camera_pose();
		if (!_Tcw.empty())
		{
			cv::Mat Rwc(3, 3, CV_32F);
			cv::Mat twc(3, 1, CV_32F);
			Rwc = _Tcw.rowRange(0, 3).colRange(0, 3).t();
			twc = -Rwc*_Tcw.rowRange(0, 3).col(3);
	
			Twc[0] = Rwc.at<float>(0, 0);
			Twc[1] = Rwc.at<float>(1, 0);
			Twc[2] = Rwc.at<float>(2, 0);
			Twc[3] = 0.0;

			Twc[4] = Rwc.at<float>(0, 1);
			Twc[5] = Rwc.at<float>(1, 1);
			Twc[6] = Rwc.at<float>(2, 1);
			Twc[7] = 0.0;

			Twc[8] = Rwc.at<float>(0, 2);
			Twc[9] = Rwc.at<float>(1, 2);
			Twc[10] = Rwc.at<float>(2, 2);
			Twc[11] = 0.0;

			Twc[12] = twc.at<float>(0);
			Twc[13] = twc.at<float>(1);
			Twc[14] = twc.at<float>(2);
			Twc[15] = 1.0;
		}
		else{
			memset((void*)Twc, 0, sizeof(float));
			Twc[0] = Twc[5] = Twc[10] = Twc[15] = 1.0f;
 		}
	}
}