
#define DEBUG_SD

class f_ship_detector: public f_base
{
protected:
	ch_image * m_pin;
	ch_vector<Rect> * m_pout;

	struct s_wf{
		int x, y;
		int d;
		s_wf():x(0),y(0),d(0){};
		s_wf(int vx, int vy, int vd):x(vx), y(vy), d(vd){};
	};

	Mat m_label, m_bin;
	int m_sd;
	int m_wait_cnt;
	vector<s_wf> m_sq;
	int m_num_depth_per_chan;
	int m_num_bins_per_chan;
	double m_alpha;
	double m_th;
	int m_th_pix;
	double *** m_hist;
	int m_num_smpls;
	unsigned int *** m_hist_tmp;
	vector<Rect> m_smplroi;
	vector<Rect> m_dtctroi; // roi for detection

public:
	f_ship_detector(const char * name): f_base(name),
		m_pin(NULL), m_pout(NULL), m_hist(NULL),
		m_hist_tmp(NULL), m_num_depth_per_chan(4),
		m_alpha(0.1), m_th(1e-8), m_sd(3), m_th_pix(15), m_wait_cnt(0)
	{
		init();
	}

	virtual ~f_ship_detector()
	{
		free();
	}

	void init()
	{
		if(m_hist != NULL)
			free();
		m_num_bins_per_chan = 1 << m_num_depth_per_chan;
		m_hist = new double**[m_num_bins_per_chan];
		m_hist_tmp = new unsigned int**[m_num_bins_per_chan];
		for(int i = 0; i < m_num_bins_per_chan; i++){
			m_hist[i] = new double*[m_num_bins_per_chan];
			m_hist_tmp[i] = new unsigned int*[m_num_bins_per_chan];
			for(int j = 0; j < m_num_bins_per_chan; j++){
				m_hist[i][j] = new double[m_num_bins_per_chan];
				m_hist_tmp[i][j] = new unsigned int[m_num_bins_per_chan];
				for(int k = 0; k < m_num_bins_per_chan; k++){
					m_hist[i][j][k] = 0.0;
					m_hist_tmp[i][j][k] = 0;
				}
			}
		}

		m_wait_cnt = 0;
	}

	void free()
	{
		if(m_hist == NULL)
			return;

		for(int i = 0; i < m_num_bins_per_chan; i++){
			for(int j = 0; j < m_num_bins_per_chan; j++){
				delete[] m_hist[i][j];
				delete[] m_hist_tmp[i][j];
			}
			delete[] m_hist[i];
			delete[] m_hist_tmp[i];
		}

		m_hist = NULL;
		m_hist_tmp = NULL;
	}

	void expand(int x, int y, int d, int ilabel);

	virtual bool check()
	{
		return m_chin[0] != NULL && m_chout[0] != NULL;
	}

	virtual bool cmd_proc(s_cmd & cmd);

	virtual bool proc();
};