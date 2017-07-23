#ifndef _CH_MAP_H_
#define _CH_MAP_H_
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// ch_map.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_map.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_map.h.  If not, see <http://www.gnu.org/licenses/>.

#include "ch_base.h"
#include "../util/aws_coord.h"
#include "../util/aws_map.h"

// contains map information - multiple layered, dynamically updatable map.
// has insert, delete, configuration methods
class ch_map: public ch_base
{
protected:
	float m_range;
	Point3f m_cecef; // x, y, z
	Point3f m_bih; // lat, lon, alt
	vector<list<const vector<Point3f>*>> m_cls;
public:
	ch_map(const char * name):ch_base(name), m_range(10000), m_cecef()
	{
	}

	bool init(int nLevels)
	{
		m_cls.resize(nLevels);
		return true;
	}

	virtual ~ch_map()
	{
		m_cls.clear();
	}

	void set_range(const float range){
		lock();
		m_range = range;
		unlock();
	}

	const float get_range()
	{
		return m_range;
	}

	void set_center(const float x, const float y, const float z)
	{
		lock();
		m_cecef.x = x;
		m_cecef.y = y;
		m_cecef.z = z;
		unlock();
	}

	void set_center_bih(const float lat, const float lon, const float alt)
	{
		lock();
		m_bih.x = lat;
		m_bih.y = lon;
		m_bih.z = alt;
		unlock();
	}

	void get_center(float & x, float & y, float & z)
	{
		lock();
		x = m_cecef.x;
		y = m_cecef.y;
		z = m_cecef.z;
		unlock();
	}

	void get_center_bih(float & lat, float & lon, float & alt)
	{
		lock();
		lat = m_bih.x;
		lon = m_bih.y;
		alt = m_bih.z;
		unlock();
	}

	void insert(const int ilevel, const vector<Point3f> * pline){
		m_cls[ilevel].push_back(pline);
	}

	void erase(const int ilevel, const vector<Point3f> * pline){
		m_cls[ilevel].remove(pline);
	}

	void clear(const int ilevel)
	{
		m_cls[ilevel].clear();
	}

	int get_num_lines(){
		return (int)m_cls.size();
	}

	const list<const vector<Point3f>*> & get_lines(const int ilevel)
	{
		return m_cls[ilevel];
	}
};

#endif