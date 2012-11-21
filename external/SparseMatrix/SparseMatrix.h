#ifndef __GSI_SPARSE_MATRIX__
#define __GSI_SPARSE_MATRIX__

#include "Vector.h"
#include <cstdio>

namespace gsi {


class SparseMatrix
{
public:
	SparseMatrix(unsigned int nRows = 1, unsigned int nCols = 1);
	SparseMatrix(const SparseMatrix & copy);
	~SparseMatrix(void);

	SparseMatrix & operator=(const SparseMatrix & copy);

	void Resize( unsigned int nRows, unsigned int nCols );

	unsigned int Rows() const;
	unsigned int Columns() const;

	void Set( unsigned int r, unsigned int c, double dValue );
	double Get( unsigned int r, unsigned int c ) const;

	void Clear(bool bFree = true);
	void ClearRow( unsigned int r );

	// fill all entries with random value  (useful for testing)
	void ToRandom(bool bSymmetric = true);

/*
 * element operator wrappers
 */ 

	class Entry
	{
	public:
		unsigned int r, c;
		SparseMatrix * pMatrix;
		double operator=(double dValue) const;
		double operator=(const Entry & e2) const;
		operator double() const;
	};
	Entry operator()(unsigned int r, unsigned int c);

	class ConstEntry
	{
	public:
		unsigned int r, c;
		const SparseMatrix * pMatrix;
		operator double() const;
	};
	ConstEntry operator()(unsigned int r, unsigned int c) const;




/*
 * row/column data structures
 */
	class Column;

	Column * GetColumn(unsigned int c);
	const Column * GetColumn(unsigned int c) const;


	class Row;

	Row * GetRow(unsigned int r);
	const Row * GetRow(unsigned int r) const;
		


/*
 * matrix information
 */
	unsigned int CountNonZeros(bool bLowerOnly = false) const;

	// returns largest non-symmetric error
	bool IsSymmetric(double dThresh = 0.0001f, double * pLargestError = NULL) const;

	// returns largest non-symmetric error
	bool IsPositiveDefinite(bool * bIsSemiDefinite = NULL) const;


/*
 * Row/Column functions 
 */
	class IColumnFunction {
	public:
		virtual void NextEntry( unsigned int r, unsigned int c, double dVal ) = 0;
	};
	void ApplyColumnFunction( unsigned int c, IColumnFunction * f ) const;


/*
 * math operations. These operations do not create any temporary matrices themselves,
 * and so force you to be as "efficient" as possible
 */
	//! set matrix to identity
	void ToIdentity();

	//! compute transpose of matrix
	void Transpose(SparseMatrix & store) const;

	//! compute this = this + B
	bool Add( const SparseMatrix & B );

	//! compute this = this - B
	bool Subtract( const SparseMatrix & B );


	//! compute C = this + B
	bool Add(const SparseMatrix & B, SparseMatrix & C ) const;

	//! compute C = this - B
	bool Subtract(const SparseMatrix & B, SparseMatrix & C ) const;


	//! compute this = val * this
	void Multiply( double dVal );


	//! compute C = this*B
	bool Multiply(const SparseMatrix & B, SparseMatrix & C) const;

	//! compute C = this*B
	bool Multiply(const Vector & B, Vector & C) const;


/*
 * inefficient but concise versions
 */
	SparseMatrix Transpose() const;
	SparseMatrix operator*(const SparseMatrix & B) const;
	SparseMatrix operator+(const SparseMatrix & B) const;
	SparseMatrix operator-(const SparseMatrix & B) const;
	SparseMatrix operator*(double dVal) const;

	Vector operator*(const Vector & B) const;



/*
 * i/o
 */
	void PrintColumns(FILE * out) const;
	void PrintRows(FILE * out) const;
	void DebugPrint( int (*printfunc)(const char * str, ...) ) const;

private:
	unsigned int m_nRows;
	unsigned int m_nCols;
	Column * m_pColumns;
	Row * m_pRows;
};


	
} // end namespace gsi

#endif // __GSI_SPARSE_MATRIX__