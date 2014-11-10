#include "stdafx.h"
#include <iostream>
#include <cstdio>

#include "aws_cminpack.h"

c_aws_lmdif::c_aws_lmdif(int am, int an):m(am), n(an),
	iwa(NULL), m_fout(NULL), m_x(NULL), dwa(NULL)
{
	lwa = m * n + 5 * n + m;

	m_x = new __cminpack_real__[n];
	m_fout = new __cminpack_real__[m];
	iwa = new int[n];
	dwa = new __cminpack_real__[lwa];

	memset(m_x, 0, sizeof(__cminpack_real__)*n);
	memset(m_fout, 0, sizeof(__cminpack_real__)*m);
	memset(iwa, 0, sizeof(int)*n);
	memset(dwa, 0, sizeof(__cminpack_real__)*lwa);
}


c_aws_lmdif::~c_aws_lmdif()
{
	delete[] m_x;
	delete[] m_fout;
	delete[] iwa;
	delete[] dwa;
}