// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef __RMS_SEGMENTATION_H__
#define __RMS_SEGMENTATION_H__
#include "config.h"
#include <IMesh.h>
#include <VFTriangleMesh.h>
#include <IDMap.h>

namespace rms {

/*
 * Represents a discrete segmentation of mesh faces
 */
class Segmentation
{
public:
	typedef unsigned int SegmentID;
	static unsigned int InvalidID;

	Segmentation(VFTriangleMesh * pMesh = NULL);

	void SetMesh(VFTriangleMesh * pMesh);
	void Clear();

	//! careful passing ID here, will break automatic segment ID generation  [TODO] fix that...
	SegmentID AppendSegment(unsigned int nSizeHint = 0, unsigned int nSetID = -1);

	template<class T>
	SegmentID AppendSegment(const T & vlist);

	void CreateSegments( const std::map<IMesh::TriangleID, int> & vSegments );

	SegmentID FindSet( IMesh::TriangleID tID );

	const std::vector<IMesh::TriangleID> & GetSet( SegmentID sID ) const;
	const std::vector<IMesh::VertexID> & GetBoundary( SegmentID sID ) const;

	void SanityCheck(VFTriangleMesh & mesh);

	void DebugRender();

protected:
	VFTriangleMesh * m_pMesh;

	static std::vector<IMesh::TriangleID> EMPTY_SET;
	static std::vector<IMesh::VertexID> EMPTY_BOUNDARY;

	struct Segment {
		SegmentID id;
		Wml::Vector3f vColor;
		std::vector<IMesh::TriangleID> vTris;
		std::vector<IMesh::VertexID> vBoundary;
		Segment() {
			id = InvalidID;  vTris.clear(); vBoundary.clear();
		}
		bool operator<( const Segment & set2 ) const {
			return id < set2.id;
		}
	};
	std::set<Segment> m_vSegments;
	typedef std::set<Segment>::iterator SegmentItr;
	typedef std::set<Segment>::const_iterator SegmentConstItr;
	Segment * FindSegment(SegmentID sID);

	std::map<IMesh::TriangleID, SegmentID> m_SegmentMap; 

	unsigned int m_nIDCounter;

public:

	class id_iterator {
	public:
		id_iterator(const id_iterator & copy) { cur = copy.cur; }

		inline Segmentation::SegmentID operator*() { 
			return (*cur).id;
		}

		inline id_iterator & operator++() {		// prefix
			++cur;
			return *this;
		}
		inline id_iterator operator++(int) {		// postfix
			id_iterator copy(*this);
			++cur;
			return copy;
		}

		inline bool operator==( id_iterator & r2 ) {
			return cur == r2.cur;
		}
		inline bool operator!=( id_iterator & r2 ) {
			return cur != r2.cur;
		}		

	protected:
		SegmentConstItr cur;

		id_iterator(const SegmentConstItr & val ) { cur = val; }
		friend class Segmentation;
	};

	id_iterator begin() const { return id_iterator(m_vSegments.begin()); }
	id_iterator end() const { return id_iterator(m_vSegments.end()); }

};



// template functions
template<class T>
Segmentation::SegmentID Segmentation::AppendSegment(const T & vlist) 
{
	SegmentID sID = AppendSegment();
	Segment * pSeg = FindSegment(sID);
	typename T::const_iterator curt(vlist.begin()), endt(vlist.end());
	while ( curt != endt ) {
		IMesh::TriangleID tID = *curt++;
		pSeg->vTris.push_back(tID);
		m_SegmentMap[tID] = pSeg->id;
	}

	return sID;
}




} // end namespace rms


#endif // __RMS_SEGMENTATION_H__
