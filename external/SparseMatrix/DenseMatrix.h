#ifndef __GSI_DENSE_MATRIX__
#define __GSI_DENSE_MATRIX__

#include "Vector.h"


namespace gsi {


class DenseMatrix
{
public:
	DenseMatrix(unsigned int nRows = 0, unsigned int nCols = 0);
	DenseMatrix(const DenseMatrix & copy);
	~DenseMatrix(void);

	DenseMatrix & operator=(const DenseMatrix & copy);

	void Resize( unsigned int nRows, unsigned int nCols, double fInitValue = 0, bool bClearFirst = true);

	unsigned int Rows() const;
	unsigned int Columns() const;

	void Set( unsigned int r, unsigned int c, double fValue );
	double Get( unsigned int r, unsigned int c ) const;

	operator double *();
	operator const double *();

	void Clear(bool bFree = true);
	void ClearRow( unsigned int r, double dValue );

/*
 * matrix information
 */
	unsigned int CountNonZeros(bool bLowerOnly = false) const;

	// returns largest non-symmetric error
	bool IsSymmetric(double dThresh = 0.0001f, double * pLargestError = NULL) const;

	// returns largest non-symmetric error
	bool IsPositiveDefinite(bool * bIsSemiDefinite = NULL) const;


/*
 * math operations. These operations do not create any temporary matrices themselves,
 * and so force you to be as "efficient" as possible
 */
	//! set matrix to identity
	void ToIdentity();

	//! compute transpose of matrix
	void Transpose(DenseMatrix & store) const;

	//! compute this = this + B
	bool Add( const DenseMatrix & B );

	//! compute this = this - B
	bool Subtract( const DenseMatrix & B );


	//! compute C = this + B
	bool Add(const DenseMatrix & B, DenseMatrix & C ) const;

	//! compute C = this - B
	bool Subtract(const DenseMatrix & B, DenseMatrix & C ) const;


	//! compute this = val * this
	void Multiply( double dVal );


	//! compute C = this*B
	bool Multiply(const DenseMatrix & B, DenseMatrix & C) const;

	//! compute C = this*B
	bool Multiply(const Vector & B, Vector & C) const;



/*
 * i/o
 */
	void Print(FILE * out) const;

protected:
	unsigned int m_nRows;
	unsigned int m_nCols;
	std::vector<double> m_vMatrix;
};


	
} // end namespace gsi

#endif // __GSI_SPARSE_MATRIX__