#ifndef __GSI_SOLVER_UMFPACK__
#define __GSI_SOLVER_UMFPACK__

#include "SparseLinearSystem.h"

namespace gsi {

/*
 * Wrapper for UMFPACK - http://www.cise.ufl.edu/research/sparse/umfpack/
 */

class Solver_UMFPACK
{
public:
	Solver_UMFPACK(SparseLinearSystem * pSystem);
	~Solver_UMFPACK(void);

	/*
	 * solve system. returns false if there were any problems (which  is not super-helpful...)
	 */
	virtual bool Solve();

	/*
	 * Factorize and then solve system 
	 */
	virtual bool Factorize();
	virtual bool Solve_Factorized();

private:
	SparseLinearSystem * m_pSystem;
};



}  // namespace gsi


#endif  // __GSI_SOLVER_UMFPACK__