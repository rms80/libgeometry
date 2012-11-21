#ifndef __GSI_SPARSE_SYMMETRIC_MATRIX_SOLVER__
#define __GSI_SPARSE_SYMMETRIC_MATRIX_SOLVER__

#include <vector>
#include <set>
#include <string>


namespace gsi {


/*
 * Wrapper for sparse symmetric matrix solving packages:
 *    TAUCS 2.2 - handles symmetric matrices - http://www.tau.ac.il/~stoledo/taucs/
 *    UMFPACK.
 */

class SparseSymmetricMatrixSolver
{
public:
	SparseSymmetricMatrixSolver(unsigned int nRows = 0, unsigned int nCols = 0);
	~SparseSymmetricMatrixSolver(void);

	void Resize( unsigned int nRows, unsigned int nCols );
	void ResizeRHS( unsigned int nRHS );

	unsigned int Rows() { return m_nRows; }
	unsigned int Columns() { return m_nCols; }

	inline void Set( unsigned int r, unsigned int c, double dValue );
	inline double Get( unsigned int r, unsigned int c );

	void Clear(bool bFree = true);
	void ClearRow( unsigned int r );

	inline void SetRHS( unsigned int r, double dValue, unsigned int nRHS = 0 );
	inline double GetRHS( unsigned int r, unsigned int nRHS = 0 );

	//! assumes dValue is at least length m_nRows (or m_nRows+1 for one-based indexing)
	inline void SetRHSVec( double * dValue, unsigned int nRHS = 0 );

	inline double GetSolution( unsigned int r, unsigned int nRHS = 0 );

	//! note - this vector is always indexed from 0...
	inline double * GetSolutionVec( unsigned int nRHS = 0 );

	/*
	 * Sanity checks
	 */
	unsigned int CountNonZeros(bool bLowerOnly = false);

	// returns largest non-symmetric error
	double IsSymmetric();


	/*
	 * permit array indices starting from 0 or 1  [default is 0]
	 */
	void SetZeroIndexMode() { m_n1Offset = 0; }
	void SetOneIndexMode() { m_n1Offset = 1; }

	enum SolverMode {
		TAUCS_LLT,					// Cholesky factorization  (symmetric positive semi-definite)
		TAUCS_LU,					// LU factorization w/ partial pivoting (out-of-core?)
		TAUCS_LDLT,					// LDLt factorization w/o pivoting
		TAUCS_MF,					// multifrontal factorization
		TAUCS_LL,					// left-looking factorization

		TAUCS_ConjGradient,			// iterative conjugate gradient algorithm  (symmetric positive definite)
		TAUCS_MinRes,				// iterative minres algorithm

		UMFPACK						// solver for non-symmetric matrices
	};

	// [default solver mode is TAUCS_MinRes]
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
		TAUCS_MaxiumuWeightBasis	// Vaidya max weight basis (symmetric matrix w/ positive diagonals)
	};
	void SetPreconditionerMode( PreconditionerMode eMode ) { m_ePreconditionerMode = eMode; }
	PreconditionerMode GetPreconditionerMode( ) { return m_ePreconditionerMode; }

	//! taucs logfile - also understands "stderr", "stdout", etc
	void SetLogFileName(const std::string & filename) { m_logfile = filename; }
	const std::string & GetLogFileName() { return m_logfile; }


	/*
	 * solve system. returns false if there were any problems (which  is not super-helpful...)
	 */
	bool Solve();

	/*
	 * Factorize system - only works for taucs...
	 */
	bool Factorize();
	bool Solve_Factorized();



private:

	struct Entry {
		unsigned int r;
		double value;
		Entry() { r = 0; value = 0; }
		Entry(unsigned int row) { r = row; value = 0; }
		Entry(unsigned int row, double val) { r = row; value = val; }
		bool operator<( const Entry & e2 ) const { return r < e2.r; }
	};
	typedef std::set<Entry> Column;
	typedef std::vector<double> RowVector;


	unsigned int m_nRows;
	unsigned int m_nCols;
	std::vector<Column> m_vColumns;

	std::vector<RowVector> m_vRHS;
	std::vector<RowVector> m_vSolutions;

	//! support one-based indexing for input values (but not output??)
	unsigned int m_n1Offset;

	SolverMode m_eSolverMode;
	OrderingMode m_eOrderMode;
	PreconditionerMode m_ePreconditionerMode;

	unsigned int m_nSolverMaxIterations;
	double m_fSolverConvergeTolerance;

	std::string m_logfile;

	bool Solve_TAUCS();
	bool Solve_UMFPACK();

	bool Factorize_TAUCS();
	bool Solve_Factor_TAUCS();
	bool Cleanup_Factor_TAUCS();
	void * m_pTaucsMatrix;
	void * m_pTaucsFactorMatrix;
	std::vector<std::string> m_vOptionStrings;
	std::vector<char *> m_vOptions;
};



}  // namespace gsi


#endif  // __GSI_SPARSE_SYMMETRIC_MATRIX_SOLVER__