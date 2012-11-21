// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)

#ifdef WIN32

#include <windows.h>
#include <gl/GL.h>
#include <gl/GLU.h>
#else
#error TODO
#endif



/*
 *
 * OpenGL debug utility functions
 *
 */


#ifdef __RMS_DEBUG_WANT_GL


#include <GL/gl.h>


static char _RMSDEBUG_GLErrorStrings[][32] = {  "GL_NO_ERROR",
												"GL_INVALID_ENUM", 
												"GL_INVALID_VALUE", 
												"GL_INVALID_OPERATION", 
												"GL_STACK_OVERFLOW",
												"GL_STACK_UNDERFLOW",
												"GL_OUT_OF_MEMORY",
												"Unknown GL Error"};

static char * glErrorString(GLenum error) 
{
	switch(error) {
		case GL_NO_ERROR: return _RMSDEBUG_GLErrorStrings[0];
		case GL_INVALID_ENUM: return _RMSDEBUG_GLErrorStrings[1];
		case GL_INVALID_VALUE: return _RMSDEBUG_GLErrorStrings[2];
		case GL_INVALID_OPERATION: return _RMSDEBUG_GLErrorStrings[3];
		case GL_STACK_OVERFLOW: return _RMSDEBUG_GLErrorStrings[4];
		case GL_STACK_UNDERFLOW: return _RMSDEBUG_GLErrorStrings[5];
		case GL_OUT_OF_MEMORY: return _RMSDEBUG_GLErrorStrings[6];
		default: return _RMSDEBUG_GLErrorStrings[7];
	}
}

static GLenum _RMSGLError = GL_NO_ERROR;

#define _RMSHaveGLError() (_RMSGLError != GL_NO_ERROR)

static bool glError()
{
	_RMSGLError = glGetError();
	int nextErr = _RMSGLError;
	while (nextErr != GL_NO_ERROR)
		nextErr = glGetError();
	return _RMSGLError != GL_NO_ERROR;
}



#endif // __RMS_DEBUG_WANT_GL
