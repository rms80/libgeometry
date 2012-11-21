#ifndef __GSI_SOLVER_TAUCS__
#define __GSI_SOLVER_TAUCS__

#include "SparseLinearSystem.h"
#include <string>

namespace gsi {

/*
 * Wrapper for UMFPACK - http://www.cise.ufl.edu/research/sparse/umfpack/
 */

class Solver_TAUCS
{
public:
	Solver_TAUCS(SparseLinearSystem * pSystem);
	~Solver_TAUCS(void);

	/*
	 * solver interface
	 */

	// solve system. returns false if there were any problems (which  is not super-helpful...)
	virtual bool Solve();

	// enable/disable storage of factorized solution, if possible 
	virtual void SetStoreFactorization(bool bEnable);
	virtual bool GetStoreFactorization() { return m_bStoreFactorization; }

	// notify that matrix changed, invalidating factorization
	virtual void OnMatrixChanged();


	/*
	 * TAUCS-specific 
	 */


	enum SolverMode {
		TAUCS_LLT,					// Cholesky factorization  (symmetric positive semi-definite)
		TAUCS_LU,					// LU factorization w/ partial pivoting (out-of-core?)
		TAUCS_LDLT,					// LDLt factorization w/o pivoting
		TAUCS_MF,					// multifrontal factorization
		TAUCS_LL,					// left-looking factorization

		TAUCS_ConjGradient,			// iterative conjugate gradient algorithm  (symmetric positive definite)
		TAUCS_MinRes,				// iterative minres algorithm
	};

	// [default solver mode is TAUCS_LLT]
	void SetSolverMode( SolverMode eMode ) { m_eSolverMode = eMode; }
	SolverMode GetSolverMode() { return m_eSolverMode; }

	//! 0 == use package default
	void SetSolverMaxIterations(unsigned int nMax) { m_nSolverMaxIterations = nMax; }
	unsigned int GetSolverMaxIterations() { return m_nSolverMaxIterations; }

	//! 0 == use package default
	void SetSolverConvergeTolerance(double fTol) { m_fSolverConvergeTolerance = fTol; }
	double GetSolverConvergeTolerance() { return m_fSolverConvergeTolerance; }

	enum OrderingMode {
		TAUCS_AUTOMATIC,			// let taucs choose an ordering
		TAUCS_IDENTITY,				// identity 
		TAUCS_GENMMD,				// multiple minimum degree (fastest, good on small to medium matrices)  (requires positive semi-def ??)
		TAUCS_MD,					// minimum degree              (from AMD package)
		TAUCS_MMD,					// multiple minimum degree     (from AMD package)
		TAUCS_AMD,					// approximate minimum degree  (from AMD package)
		TAUCS_METIS,				// METIS library code (fast, better on large matrices)
		TAUCS_TREEORDER,			// no-fill ordering code for matrices whose graphs are trees (??)
		TAUCS_COLAMD				// column approximate minimum-degree (for LU factorization)
	};

	// [default ordering mode is TAUCS_AUTOMATIC]
	void SetOrderingMode( OrderingMode eMode ) { m_eOrderMode = eMode; }
	OrderingMode GetOrderingMode() { return m_eOrderMode; }


	enum PreconditionerMode {
		NoPreconditoner,
		TAUCS_MaximumWeightBasis	// Vaidya max weight basis (symmetric matrix w/ positive diagonals)
	};
	void SetPreconditionerMode( PreconditionerMode eMode ) { m_ePreconditionerMode = eMode; }
	PreconditionerMode GetPreconditionerMode( ) { return m_ePreconditionerMode; }


	//! taucs logfile - also understands "stderr", "stdout", etc
	void SetLogFileName(const std::string & filename) { m_logfile = filename; }
	const std::string & GetLogFileName() { return m_logfile; }


private:
	SparseLinearSystem * m_pSystem;

	bool m_bStoreFactorization;

	SolverMode m_eSolverMode;
	OrderingMode m_eOrderMode;
	PreconditionerMode m_ePreconditionerMode;

	unsigned int m_nSolverMaxIterations;
	double m_fSolverConvergeTolerance;

	std::string m_logfile;

	std::vector<std::string> m_vOptionStrings;
	std::vector<char *> m_vOptions;
	bool init_options();

	void * m_pTaucsMatrix;
	bool allocate_taucs_matrix();
	void free_taucs_matrix();

	std::vector<double> m_vRHS;
	std::vector<double> m_vSolution;

	bool Solve_Direct();

	void * m_pTaucsFactorMatrix;
	bool Compute_Factorization();
	bool Solve_Factorization();
};



}  // namespace gsi


#endif  // __GSI_SOLVER_TAUCS__