// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// ch_vector.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_vector.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_vector.h.  If not, see <http://www.gnu.org/licenses/>. 

template<class T> class ch_vector: public ch_base
{
protected:
	vector<T*> m_objs;
	
public:
	ch_vector(const char * name):ch_base(name){};
	virtual ~ch_vector(){};
	void push(T * pobj){
		lock();
		m_objs.push_back(pobj);
		unlock();
	}

	T * pop(){
		T * pobj = NULL;
		if(m_objs.size()){
			lock();
			pobj = m_objs.back();
			m_objs.pop_back();
			unlock();
		}

		return pobj;
	}

	virtual void tran(){
		return;
	}
};
