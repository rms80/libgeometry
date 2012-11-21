#ifndef __GSI_VECTOR__
#define __GSI_VECTOR__

namespace gsi {

/*
 * basic dense vector class
 */

class Vector
{
public:
	Vector(size_t nSize = 0);
	Vector(const Vector & copy);
	~Vector(void);

	const Vector & operator=(const Vector & copy);

	void Resize( size_t nSize );
	size_t Size() const;
	void Set( unsigned int i, double dValue );
	double Get( unsigned int i ) const;

	//! this resets vector to 0
	void Clear();

	//! returns pointers into std::vector - will change if vector is resized!!
	double * GetValues();
	const double * GetValues() const;

	const double & operator[](unsigned int i) const;
	double & operator[](unsigned int i);

private:
	double * m_pValues;
	size_t m_nSize;
};



} // end namespace gsi

#endif // __GSI_VECTOR__