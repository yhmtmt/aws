class ch_nmea: public ch_base
{
protected:
	int m_max_buf;
	int m_head;
	int m_tail;
	char ** m_buf;
	bool alloc(int size){
		m_buf = new char * [size];
		if(!m_buf)
			return false;

		char * p = new char [83 * size];
		if(!p){
			delete[] m_buf;
			m_buf = NULL;
			return false;
		}

		for(int i = 0; i < size; i++, p+=83){
			m_buf[i] = p;
		}
		return true;
	}

	void release(){
		if(m_buf){
			if(m_buf[0])
				delete[] m_buf[0];
			delete[] m_buf;
		}
		m_buf = NULL;
	}

public:
	ch_nmea(const char * name): ch_base(name), m_max_buf(128), 
		m_head(0), m_tail(0)
	{
		alloc(m_max_buf);
	}

	virtual ~ch_nmea()
	{
		release();
	}

	bool pop(char * buf)
	{
		lock();
		if(m_head == m_tail){
			unlock();
			return false;

		}
		char * p = m_buf[m_head];
		for(;*p != '\0'; p++, buf++){
			*buf = *p;
		}
		*buf = *p;
		m_head++;
		m_head %= m_max_buf;
		unlock();
		return true;
	}

	bool push(const char * buf)
	{
		lock();
		int next_tail = (m_tail + 1) % m_max_buf;
		if(m_head == next_tail){
			unlock();
			return false;
		}
		char * p = m_buf[m_tail];
		for( ;*buf != '\0'; buf++, p++){
			*p = *buf;
		}
		*p = *buf;
		m_tail = next_tail;
		unlock();
		return true;
	}

	virtual void tran()
	{
	}
};

class ch_ship: public ch_base
{
private:

};