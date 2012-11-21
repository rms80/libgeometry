// Ryan Schmidt - rms@unknownroad.com
// Copyright (c) 2006-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt


// #include "winstuff.h"
#include "Texture.h"

#ifndef WIN_32
  #include <GL/gl.h>
  #include <GL/glu.h>
  #include <GL/glut.h>
#else
#include <gl/GL.h>
#include <gl/GLU.h>
#endif


#include <limits>
#include <rmsdebug.h>
#include <config.h>

Texture::TextureID Texture::INVALID_TEXTURE = std::numeric_limits<unsigned int>::max();
Texture::TextureID Texture::m_sTextureIDGen = 0;
Texture::TextureID Texture::GenTextureID() { 
	return m_sTextureIDGen++;
}

Texture::Texture(TextureID nID)
{
	m_nOpenGLTextureID = 0;
	m_nDeleteThisOpenGLTextureID = 0;
	m_nWidth = m_nHeight = 0;
	m_nTexWidth = m_nTexHeight = 0;
	m_fScaleX = m_fScaleY = 1.0f;
	m_nGLTextureType = GL_TEXTURE_2D;
	m_nGLTextureEnvMode = GL_MODULATE;
	m_nGLTextureFilter = GL_LINEAR;
	m_nGLTextureWrapMode = GL_CLAMP;
	if ( nID == INVALID_TEXTURE )
		m_nTextureID = GenTextureID();
	else
		m_nTextureID = nID;

}

Texture::Texture( const Texture * pCopy )
{
	m_nWidth = pCopy->m_nWidth;
	m_nHeight = pCopy->m_nHeight;
	m_vBuffer = pCopy->m_vBuffer;
	m_nComponents = pCopy->m_nComponents;
	m_nGLTextureFormat = pCopy->m_nGLTextureFormat;
	m_nGLTextureType = pCopy->m_nGLTextureType;
	m_nGLTextureEnvMode = pCopy->m_nGLTextureEnvMode;
	m_nGLTextureFilter = pCopy->m_nGLTextureFilter;
	m_nGLTextureWrapMode = pCopy->m_nGLTextureWrapMode;
	m_nTexWidth = pCopy->m_nTexWidth;
	m_nTexHeight = pCopy->m_nTexHeight;
	m_fScaleX = pCopy->m_fScaleX;
	m_fScaleY = pCopy->m_fScaleY;

	m_nOpenGLTextureID = 0;
	m_nTextureID = GenTextureID();
}


Texture::~Texture()
{
	FreeTexture();
}


const unsigned char * Texture::GetPixel( unsigned int x, unsigned int y )
{
	return &m_vBuffer[ y * m_nWidth * m_nComponents + x * m_nComponents ];
}

const unsigned char * Texture::GetPixel( float fUnitX, float fUnitY )
{
	unsigned int xi = (unsigned int)(fUnitX * (float)m_nWidth);  if ( xi >= m_nWidth ) xi = m_nWidth-1;
	unsigned int yi = (unsigned int)(fUnitY * (float)m_nHeight); if ( yi >= m_nHeight ) yi = m_nHeight-1;
	return GetPixel(xi,yi);
}

void Texture::GetTexCoords( float fUnitX, float fUnitY, unsigned int & xi, unsigned int & yi )
{
	xi = (unsigned int)(fUnitX * (float)m_nWidth);  if ( xi >= m_nWidth ) xi = m_nWidth-1;
	yi = (unsigned int)(fUnitY * (float)m_nHeight); if ( yi >= m_nHeight ) yi = m_nHeight-1;
}

void Texture::InitializeBuffer( unsigned int nWidth, unsigned int nHeight, 
							    const std::vector<unsigned char> & vBuffer,
								unsigned int nGLTextureFormat )
{
	FreeTexture();

	m_nWidth = nWidth;
	m_nHeight = nHeight;
	m_vBuffer = vBuffer;

	//  figure out scaling factors
	m_nTexWidth = 1;
	while ( m_nTexWidth < nWidth )
		m_nTexWidth *= 2;
	float fScaleX = (float)m_nTexWidth / (float)nWidth;
	m_nTexHeight = 1;
	while ( m_nTexHeight < nHeight )
		m_nTexHeight *= 2;
	float fScaleY = (float)m_nTexHeight / (float)nHeight;
	m_fScaleX = (fScaleX < fScaleY) ? fScaleX : fScaleY;
	m_fScaleX = 1.0f / m_fScaleX;	// scale tex coords *down* to scale image *up*

	lgASSERT( (unsigned int)vBuffer.size() % (nWidth * nHeight) == 0 );
	m_nComponents = (unsigned int)vBuffer.size() / (nWidth * nHeight);

	m_nGLTextureFormat = nGLTextureFormat;
}

void Texture::SetTextureInfo( unsigned int nGLTextureType, 
							  unsigned int nGLTextureFilter,
							  unsigned int nGLTextureWrapMode )
{
	m_nGLTextureType = nGLTextureType;
	m_nGLTextureFilter = nGLTextureFilter;
	m_nGLTextureWrapMode = nGLTextureWrapMode;
}


unsigned int Texture::GLTextureID()
{
	if ( m_nOpenGLTextureID != 0 )
		return m_nOpenGLTextureID;

	//  delete old texture ID if we don't want it anymore
	if ( m_nDeleteThisOpenGLTextureID != 0 ) {
		glDeleteTextures(1, &m_nDeleteThisOpenGLTextureID);
		m_nDeleteThisOpenGLTextureID = 0;
	}

	glGenTextures(1, &m_nOpenGLTextureID);

	glEnable(m_nGLTextureType);
	glBindTexture( m_nGLTextureType, m_nOpenGLTextureID );

	glTexParameteri( m_nGLTextureType, GL_TEXTURE_MIN_FILTER, m_nGLTextureFilter );
	glTexParameteri( m_nGLTextureType, GL_TEXTURE_MAG_FILTER, m_nGLTextureFilter );

	glTexParameteri( m_nGLTextureType, GL_TEXTURE_WRAP_S, m_nGLTextureWrapMode );
	glTexParameteri( m_nGLTextureType, GL_TEXTURE_WRAP_T, m_nGLTextureWrapMode );
//	glTexParameteri( m_nGLTextureType, GL_TEXTURE_WRAP_R, nGLTextureWrapMode );

	int nBorder = 0;
	glTexImage2D( m_nGLTextureType, 0, m_nGLTextureFormat, m_nTexWidth + 2*nBorder, m_nTexHeight + 2*nBorder, nBorder,
				  m_nGLTextureFormat, GL_UNSIGNED_BYTE, NULL ); //&m_vBuffer[0] );

	int xoff = (m_nTexWidth - m_nWidth)/2;
	int yoff = (m_nTexHeight - m_nHeight)/2;
	glTexSubImage2D( m_nGLTextureType, 0, xoff, yoff, m_nWidth, m_nHeight, m_nGLTextureFormat, GL_UNSIGNED_BYTE, &m_vBuffer[0] );

	glBindTexture(m_nGLTextureType, 0);
	glDisable(m_nGLTextureType);

	return m_nOpenGLTextureID;
}


void Texture::Enable()
{
	unsigned int nTextureID = GLTextureID();
	if ( nTextureID != 0 ) {
		glEnable( m_nGLTextureType );
		glBindTexture( m_nGLTextureType, nTextureID );
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, m_nGLTextureEnvMode);

		// apply texture scaling to get image into the right position...
		glPushAttrib( GL_TRANSFORM_BIT );
		glMatrixMode(GL_TEXTURE);
		glPushMatrix();
		glLoadIdentity();
		glTranslatef( 0.5f, 0.5f, 0.0f );
		glScalef( m_fScaleX, m_fScaleX, 0.0f );
		glTranslatef( -0.5f, -0.5f, 0.0f );
		glPopAttrib();

	}
}

void Texture::Disable()
{
	glBindTexture( m_nGLTextureType, 0 );
	glDisable( m_nGLTextureType );

	glPushAttrib( GL_TRANSFORM_BIT );
	glMatrixMode(GL_TEXTURE);
	glPopMatrix();
	glPopAttrib();
}

void Texture::FreeTexture()
{
/*
	if ( m_nOpenGLTextureID != 0 ) {
		if ( wglGetCurrentContext() != NULL )
			glDeleteTextures(1, &m_nOpenGLTextureID);
		else
			m_nDeleteThisOpenGLTextureID = m_nOpenGLTextureID;
	}
	m_nOpenGLTextureID = 0;
	*/
}



void Texture::ReplaceColorAlpha( unsigned char nRed, unsigned char nBlue, unsigned char nGreen, unsigned char nNewAlpha )
{
	if ( m_nComponents != 4 )
		return;
	unsigned int nAlphaChannel = m_nComponents - 1;

	for ( unsigned int yi = 0; yi < m_nHeight; ++yi ) {
		for ( unsigned int xi = 0; xi < m_nWidth; ++xi ) {
			unsigned int nIndex = (yi * m_nWidth * m_nComponents) + (xi * m_nComponents);

			if ( m_vBuffer[nIndex+0] == nRed && m_vBuffer[nIndex+1] == nBlue && m_vBuffer[nIndex+2] == nGreen )
				m_vBuffer[nIndex+nAlphaChannel] = nNewAlpha;
		}
	}
}

void Texture::SetEdgeAlpha( unsigned char nSetAlpha )
{
	unsigned int nAlphaChannel = m_nComponents - 1;

	for ( unsigned int yi = 0; yi < m_nHeight; ++yi ) {
		bool bEdgeY = (yi == 0 || yi == m_nHeight-1);
		for ( unsigned int xi = 0; xi < m_nWidth; ++xi ) {
			unsigned int nIndex = (yi * m_nWidth * m_nComponents) + (xi * m_nComponents);

			bool bEdgeX = bEdgeY ||  (xi == 0 || xi == m_nWidth-1 );
			if (bEdgeX)
				m_vBuffer[ nIndex + nAlphaChannel ] = nSetAlpha;
		}
	}	
}


void Texture::FlipY()
{
	unsigned char tmp[4];

	for ( unsigned int yi = 0; yi < m_nHeight/2; ++yi ) {
		unsigned int nRow1 = (yi * m_nWidth * m_nComponents);
		unsigned int nRow2 = ( (m_nHeight-yi-1)  * m_nWidth * m_nComponents );

		for ( unsigned int xi = 0; xi < m_nWidth; ++xi ) {
			unsigned int nElem1 = nRow1 + (xi * m_nComponents );
			unsigned int nElem2 = nRow2 + (xi * m_nComponents );

			memcpy( tmp, &m_vBuffer[ nElem1 ], m_nComponents * sizeof(unsigned char) );
			memcpy( &m_vBuffer[nElem1], &m_vBuffer[nElem2], m_nComponents * sizeof(unsigned char) );
			memcpy( &m_vBuffer[nElem2], tmp, m_nComponents * sizeof(unsigned char) );

		}

	}
}


void Texture::MakeCheckerImage( int nTexSize, int nCheckerSize, std::vector<unsigned char> & vTexBuf, bool bAlpha )
{
	// make texture
	vTexBuf.resize(0);
	for ( int h = 0; h < nTexSize; ++h ) {
		bool bYOdd = ((h / nCheckerSize) % 2 == 1);
		for ( int w = 0; w < nTexSize; ++w ) {
			bool bXOdd = ((w / nCheckerSize) % 2 == 1);
			bool bBlack = (!bYOdd && bXOdd) || (bYOdd && !bXOdd);
			unsigned char nByte = (bBlack) ? 0 : 255;
			vTexBuf.push_back(nByte);
			vTexBuf.push_back(nByte);
			vTexBuf.push_back(nByte);
			if ( bAlpha )
				vTexBuf.push_back(255);
		}
	}
}

	// buffer rgb is channel map [0,1,2], buffer bgr is channel map [2,1,0]
void Texture::MakeFromBuffer( unsigned int nImageWidth, unsigned int nImageHeight, unsigned int nImageChannels,
							  const unsigned char * pBuffer,
						      const unsigned int nChannelMap[3], bool bAddAlpha, std::vector<unsigned char> & vTexBuf,
							  unsigned int & nTexWidth, unsigned int & nTexHeight )
{
	unsigned int nImageSize = nImageWidth * nImageHeight * nImageChannels;
	unsigned int nTexChannels = nImageChannels;
	if ( nTexChannels < 4 && bAddAlpha ) 
		nTexChannels++;

	nTexWidth = nImageWidth;
	nTexHeight = nImageHeight;
	unsigned int nTexSize = nTexWidth * nTexHeight * nTexChannels;

	vTexBuf.resize(nTexSize);

	for ( unsigned int i = 0; i < nImageSize/nImageChannels; ++i ) {
		unsigned int nImgIndex = i * nImageChannels;
		unsigned int nTexIndex = i * nTexChannels;

		vTexBuf[nTexIndex + nChannelMap[0]] = (unsigned char)pBuffer[nImgIndex ];
		if ( nImageChannels > 1 )
			vTexBuf[nTexIndex + nChannelMap[1]] = (unsigned char)pBuffer[nImgIndex +1];
		if ( nImageChannels > 2 )
			vTexBuf[nTexIndex + nChannelMap[2]] = (unsigned char)pBuffer[nImgIndex +2];
		
		if  ( nTexChannels == 4 || bAddAlpha ) {
			if ( nImageChannels == 4 )
				vTexBuf[ nTexIndex + (nTexChannels-1) ] = (unsigned char)pBuffer[nImgIndex +3];
			else
				vTexBuf[ nTexIndex + (nTexChannels-1) ] = 255;
		}
	}

}





TextureSet::TextureSet()
{
}

void TextureSet::AddTexture( Texture * pTexture )
{
	lgASSERT(pTexture->ID() != Texture::INVALID_TEXTURE);
	m_vTextures[pTexture->ID()] = pTexture;

	m_vOrderedList.insert( pTexture->ID() );
}

void TextureSet::RemoveTexture( Texture * pTexture )
{
	lgASSERT(pTexture->ID() != Texture::INVALID_TEXTURE);

	std::map<Texture::TextureID, Texture *>::iterator found( 
		m_vTextures.find(pTexture->ID()) );
	if ( found != m_vTextures.end() )
		m_vTextures.erase( found );

	m_vOrderedList.erase( pTexture->ID() );
}

Texture * TextureSet::GetTexture( Texture::TextureID nID )
{
	if ( m_vTextures.find( nID ) != m_vTextures.end() )
		return m_vTextures.find(nID)->second;
	else
		return NULL;
}

Texture * TextureSet::GetTextureByIndex( unsigned int nIndex )
{
	std::set< Texture::TextureID >::iterator cur( m_vOrderedList.begin()), end(m_vOrderedList.end());
	unsigned int i = 0;
	Texture::TextureID nEntry = Texture::INVALID_TEXTURE;
	while ( cur != end ) {
		if ( i == nIndex ) {
			nEntry = *cur;
			cur = end;
		}
		++i;
		++cur;
	}

	if ( nEntry == Texture::INVALID_TEXTURE )
		return NULL;
	else
		return GetTexture( nEntry );
}
