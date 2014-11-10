#pragma comment(lib, "cminpack_dll.lib")
//cminpack.lib

//#include "cminpack/include/minpack.h"
#include "cminpack.h"

// * Specify the output dimension and the input dimension to the constructor
// * Set initial input values using method x(int).
// * Call optimize() with call back function defined as follows.
// int func(void * p, int m, int n, const __cminpack_real__ *x,
//			__cminpack_real__ *fvec, int iflag);
// - p is the same parameter as that in the parameter list of optimize()
// - m and n is the output and input dimension
// - x and fvec is the input and output vectors
// - iflag is indicates the execution mode of the callback. (usually we dont use it)

class c_aws_lmdif
{
private:
	int m /* number of output */, n /* number of input */;
	int lwa;
	int *iwa;
	__cminpack_real__ *m_fout, *m_x, *dwa;
public:
	c_aws_lmdif(int am, int an);
	~c_aws_lmdif();

	int get_dim_x(){ return n; };
	int get_dim_fout(){ return m; };
	__cminpack_real__ & x(int i){ return m_x[i];};
	__cminpack_real__ & fout(int j){return m_fout[j];};

	int optimize(cminpack_func_mn func, void * p, __cminpack_real__ tol)
	{
		return lmdif1(func, p, m, n, m_x, m_fout, tol, iwa, dwa, lwa);
	}
};
