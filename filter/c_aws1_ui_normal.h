// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// c_aws1_ui_normal.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_aws1_ui_normal.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_aws1_ui_normal.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _C_AWS1_UI_NORMAL_H_
#define _C_AWS1_UI_NORMAL_H_

class c_aws1_ui_normal: public c_aws1_ui_core
{
protected:
  // for keyboard control
  int m_num_ctrl_steps;
  enum e_eng_ctrl{
    EC_MAIN, EC_SUB
  } m_ec;

  unsigned char * m_rud_pos;
  unsigned char * m_meng_pos;
  unsigned char * m_seng_pos;

  float m_rud_aws_f;
  float m_meng_aws_f;
  float m_seng_aws_f;

  unsigned char step_down(unsigned char val, unsigned char * vpos){
    int i = m_num_ctrl_steps;
    int iup = 2 * m_num_ctrl_steps, idown = 0;
    do{
      if(vpos[i] < val){
	idown = i;
      }else if(vpos[i] > val){
	iup = i;
      }else{
	return vpos[max(i - 1, 0)];
      }

      i = (idown + iup) >> 1;
      if(i == idown){
	return vpos[idown];
      }
    }while(1);

    return vpos[m_num_ctrl_steps];
  }

  unsigned char step_up(unsigned char val, unsigned char * vpos){
    int i = m_num_ctrl_steps;
    int iup = 2 * m_num_ctrl_steps, idown = 0;
    do{
      if(vpos[i] < val){
	idown = i;
      }else if(vpos[i] > val){
	iup = i;
      }else{
	return vpos[min(i + 1, 2 * m_num_ctrl_steps)];
      }

      i = (idown + iup) >> 1;
      if(i == idown){
	return vpos[iup];
      }
    }while(1);

    return vpos[m_num_ctrl_steps];
  }

public:
	c_aws1_ui_normal(f_aws1_ui * _pui):c_aws1_ui_core(_pui), m_num_ctrl_steps(4), m_ec(EC_MAIN),
		m_rud_pos(NULL), m_meng_pos(NULL), m_seng_pos(NULL), m_rud_aws_f(127.), m_meng_aws_f(127.),
		m_seng_aws_f(127.)
	{
		// allocate control positions
		m_rud_pos = new unsigned char[m_num_ctrl_steps * 2 + 1];
		m_meng_pos = new unsigned char[m_num_ctrl_steps * 2 + 1];
		m_seng_pos = new unsigned char[m_num_ctrl_steps * 2 + 1];

		double stepf, stepb;
		double sumf, sumb;
		m_rud_pos[m_num_ctrl_steps] = 127;
		stepf = (double) (255 - 127) / (double) m_num_ctrl_steps;
		stepb = (double) (127 - 0) / (double) m_num_ctrl_steps;
		sumf = sumb = 127.;
		for(int i = 1; i < m_num_ctrl_steps; i++){
			sumf += stepf;
			sumb -= stepb;
			m_rud_pos[m_num_ctrl_steps - i] = (unsigned char) sumb;
			m_rud_pos[m_num_ctrl_steps + i] = (unsigned char) sumf;
		}
		m_rud_pos[m_num_ctrl_steps * 2] = 255;
		m_rud_pos[0] = 0;

		stepf = (double) (255 - 127 - 25) / (double) (m_num_ctrl_steps - 1);
		stepb = (double) (127 - 25 - 0) / (double) (m_num_ctrl_steps - 1);
		sumf = sumb = 127.;

		m_meng_pos[m_num_ctrl_steps] = 127;
		m_meng_pos[m_num_ctrl_steps+1] = 127 + 25;
		m_meng_pos[m_num_ctrl_steps-1] = 127 - 25;

		m_seng_pos[m_num_ctrl_steps] = 127;
		m_seng_pos[m_num_ctrl_steps+1] = 127 + 25;
		m_seng_pos[m_num_ctrl_steps-1] = 127 - 25;

		sumf = 127 + 25;
		sumb = 127 - 25;
		for (int i = 2; i < m_num_ctrl_steps; i++){
			sumf += stepf;
			sumb -= stepb;
			m_meng_pos[m_num_ctrl_steps + i] = m_seng_pos[m_num_ctrl_steps + i] = saturate_cast<unsigned char>(sumf);
			m_meng_pos[m_num_ctrl_steps - i] = m_seng_pos[m_num_ctrl_steps - i] = saturate_cast<unsigned char>(sumb);
		}

		m_meng_pos[m_num_ctrl_steps * 2] = m_seng_pos[m_num_ctrl_steps * 2] = 255;
		m_meng_pos[0] = m_seng_pos[0] = 0;

	}

	~c_aws1_ui_normal()
	{
		delete[] m_rud_pos;
		delete[] m_meng_pos;
		delete[] m_seng_pos;

		m_rud_pos = NULL;
		m_meng_pos = NULL;
		m_seng_pos = NULL;
	}

	void set_ctrl(unsigned char rud, unsigned char meng, unsigned char seng)
	{
		m_rud_aws_f = rud;
		m_meng_aws_f = meng;
		m_seng_aws_f = seng;
	}

	virtual void js(const s_jc_u3613m  & js);
	virtual void draw();
	virtual void key(int key, int scancode, int action, int mods);
};

#endif

