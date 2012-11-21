#ifndef __GSI_SPARSE_LINEAR_SYSTEM__
#define __GSI_SPARSE_LINEAR_SYSTEM__

#include <vector>

#include "SparseMatrix.h"

namespace gsi {


class SparseLinearSystem
{
public:
	SparseLinearSystem(unsigned int nRows = 0, unsigned int nColumns = 0);
	~SparseLinearSystem(void);

	void Resize( unsigned int nRows, unsigned int nColumns );

	const SparseMatrix & Matrix() const;
	void SetMatrix( const SparseMatrix & vSet );

	void Set( unsigned int r, unsigned int c, double dValue );
	double Get( unsigned int r, unsigned int c ) const;

	void ResizeRHS( unsigned int nRHS, bool bSetToZero = true );
	unsigned int NumRHS() const;

	bool SetRHS( unsigned int iRHS, const Vector & vSet );
	void SetRHS( unsigned int i, double dValue, unsigned int iRHS = 0 );

	const Vector & GetRHS( unsigned int iRHS ) const;
	double GetRHS( unsigned int i, unsigned int iRHS = 0 ) const;

	const Vector & GetSolution( unsigned int iRHS ) const;
	double GetSolution( unsigned int i, unsigned int iRHS = 0 ) const;

	bool MultiplyRHS( const SparseMatrix & vMatrix );


	//! [WARNING] if you clear or resize this matrix, the size of the RHS and Solution vectors could become out-of-sync.
	//    (it is much safer to work on copies or use the functions above)
	SparseMatrix & Matrix();
	Vector & GetRHS( unsigned int iRHS );
	Vector & GetSolution( unsigned int iRHS );


	/*
	 * solve system. returns false if there were any problems (which  is not super-helpful...)
	 */
	bool Solve();

	/*
	 * try to pre-factorize system, if possible
	 */
	bool Factorize();
	void ClearFactorization();


protected:
	SparseMatrix m_matrix;

	std::vector<Vector> m_vRHS;
	std::vector<Vector> m_vSolutions;
};




}  // namespace gsi


#endif  // __GSI_SPARSE_LINEAR_SYSTEM__