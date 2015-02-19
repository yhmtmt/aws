#ifndef _F_DS_VFILE_H_
#define _F_DS_VFILE_H_
// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_ds_video.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ds_video.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ds_video.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "f_ds_video.h"

class f_ds_vfile:public f_ds_video
{
	ch_image * m_pout;
	IBaseFilter * m_phsplt; // haali splitter ar
	IBaseFilter * m_pffdec; // ffdshow decoder
	char m_fname[1024];
	char m_start_time_str[64];
	wchar_t * m_fnamew;
	long long m_start_time;
	LONGLONG m_duration;
	bool m_oneshot;
	bool m_bshot;
	long long m_cur_time_shot;
public:
	f_ds_vfile(const char * name);
	virtual ~f_ds_vfile();

	bool open(const char * fname);
	virtual void close();
	virtual bool init_run(){
		if(!f_ds_video::init_run())
			return false;

		if(m_chout.size() != 1)
			return false;

		m_pout = dynamic_cast<ch_image*>(m_chout[0]);
		if(m_pout == NULL)
			return false;

		return 	open(m_fname);
	}

	virtual void destroy_run()
	{
		close();
	}

	virtual bool seek(long long seek_time);
	virtual bool run(long long start_time, long long end_time);
	virtual bool stop();
	virtual bool proc(){
		
		if(is_pause()){
			OAFilterState ofs;
			m_pControl->GetState(INFINITE, &ofs);
			if(ofs == State_Running)
				m_pControl->Pause();
			return true;
		}else{
			OAFilterState ofs;
			m_pControl->GetState(INFINITE, &ofs);
			if(ofs == State_Paused){	
				long long pos = m_cur_time - m_start_time;
				
				m_pSeek->SetPositions(&pos,
					AM_SEEKING_AbsolutePositioning,
					&m_duration, AM_SEEKING_AbsolutePositioning);
				
				m_pControl->Run();
			}
			do{
				if(m_pControl->GetState(INFINITE, &ofs) != S_OK){
					continue;
				}
			}while(ofs != State_Running);
		}
		
		Mat img;
		m_bstream = grab(img);
		if(!m_bstream)
			return true;

		m_time_shot = m_cur_time;
		m_pout->set_img(img, m_cur_time);
		return true;
	}

	virtual bool grab(Mat & img);
	virtual bool cmd_proc(s_cmd & cmd);
};

#endif