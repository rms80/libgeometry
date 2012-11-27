// Ryan Schmidt - rms@unknownroad.com
// Copyright (c) 2006-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt

// 

#include "MeshObject.h"
#include "ExtendedWmlCamera.h"
#include <rmsprofile.h>

#ifdef WIN_32
  #include "winstuff.h"
  #include "glut.h"
  #include <gl/GL.h>
  #include <gl/GLU.h>
#else
  #include <GL/gl.h>
  #include <GL/glu.h>
  #include <GL/glut.h>
#endif

MeshObject g_mesh;
Wml::ExtendedWmlCamera * g_pCamera = NULL;

// input modes for mouse handler
enum INPUT_MODE {
	NONE,
	DOLLY,
	ORBIT,
	PAN,
	MOVE_EXPMAP_SEED,
	SCALE_EXPMAP,
	ROTATE_EXPMAP
};
INPUT_MODE g_inputMode = NONE;


// display layout modes
enum DISPLAY_MODE {
	MESH,
	MESH_AND_QUARTERUV,
	UVONLY,

	LAST_MODE		// leave at the end!
};
DISPLAY_MODE g_displayMode = MESH_AND_QUARTERUV;


// input device state
bool g_bAltButtonDown = false;
bool g_bCtrlButtonDown = false;
bool g_bLeftMouseDown = false;
bool g_bMiddleMouseDown = false;
bool g_bRightMouseDown = false;
Wml::Vector2f g_vLastMousePos = Wml::Vector2f::ZERO;


void 
drawString (char *s)
{
  unsigned int i;
  for (i = 0; i < strlen (s); i++)
    glutBitmapCharacter (GLUT_BITMAP_HELVETICA_10, s[i]);
};


// this is (well, should be) the only platform-dependent bit of code...
void LoadNewMesh(const char * pFilename)
{
	std::string sFilename = std::string(pFilename);
	g_mesh.ReadMeshOBJ(sFilename.c_str());
}



void glut_initgl()
{
	g_pCamera = new Wml::ExtendedWmlCamera(1.0f,1.0f);
	// set default frame (lookat)
	g_pCamera->SetTargetFrame( Wml::Vector3f(0.0, 0.0, 5.0f), 
		Wml::Vector3f(-1.0f, 0.0, 0.0),
		Wml::Vector3f(0.0, 1.0f, 0.0), 
		Wml::Vector3f(0.0, 0.0, 0.0));

	// enable color material
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

	// initialize depth test
	glEnable(GL_DEPTH_TEST);

	// enable lighting
	glEnable(GL_LIGHTING);

	// set ambient light
	GLfloat ambient[] = { 0.1f, 0.1f, 0.1f, 1.0f };
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

	// enable 2 lights
	glEnable(GL_LIGHT0);
	float fEye = (g_pCamera->GetLocation() - g_pCamera->GetTarget()).Length();
	float vUpLeft[4] = {-2.0f*fEye, fEye, fEye, 1.0f};
	glLightfv(GL_LIGHT0, GL_POSITION, vUpLeft);
	float fAmbient1[4] = {0,0,0,1};
	glLightfv(GL_LIGHT0, GL_AMBIENT, fAmbient1);
	float fDiffuse1[4] = {0.6f, 0.6f, 0.6f, 1.0f};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, fDiffuse1);
	float fSpec1[4] = {0.5f, 0.5f, 0.5f,1.0f};
	glLightfv(GL_LIGHT0, GL_SPECULAR, fSpec1);

	glEnable(GL_LIGHT1);
	float vUpRight[4] = { 2.0f*fEye, fEye, fEye, 1.0f};
	glLightfv(GL_LIGHT1, GL_POSITION, vUpRight);
	float fAmbient2[4] = {0,0,0,1};
	glLightfv(GL_LIGHT1, GL_AMBIENT, fAmbient2);
	float fDiffuse2[4] = {0.5f, 0.5f, 0.5f, 1.0f};
	glLightfv(GL_LIGHT1, GL_DIFFUSE, fDiffuse2);
	float fSpec2[4] = {0.5f, 0.5f, 0.5f,1.0f};
	glLightfv(GL_LIGHT1, GL_SPECULAR, fSpec2);

	// set fixed specular material color
	float fSpecular[] = {1.0f, 1.0f, 1.0f, 1.0f};
	glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, fSpecular );
	glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, 40.0f );
}



void glut_main_display(void)
{
	// make sure expmap is computed

  _RMSTUNE_start(0);
	g_mesh.ValidateExpMap();
_RMSTUNE_end(0);
_RMSTUNE_Print_Time("califate",0);


	// clear window
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	// render the 3D view
	if ( g_displayMode == MESH || g_displayMode == MESH_AND_QUARTERUV ) {
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		g_pCamera->Update();

		g_mesh.Render();

		glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT);
		glDisable(GL_LIGHTING);
		glColor3f(0.0f,1.0f,0.0f);
		glPointSize(10.0f);
		glBegin(GL_POINTS);
		glVertex3fv( g_mesh.ExpMapFrame().Origin() );
		glEnd();
		glPopAttrib();
	}


	if ( g_displayMode == MESH_AND_QUARTERUV || g_displayMode == UVONLY ) {

		// render the 2D overlay
		const unsigned int * viewport = g_pCamera->GetViewport();
		int nDivider = (g_displayMode == MESH_AND_QUARTERUV) ? 2 : 1;
		int nMinDim = std::min(viewport[2], viewport[3]);
		glViewport(0, 0, nMinDim/nDivider, nMinDim/nDivider);
		
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(-1, 1, -1, 1);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		g_mesh.RenderParamMesh();
	}


	glutSwapBuffers();
};




void glut_main_reshape (int cx, int cy)
{
	// set viewport and frustum, and update camera
	g_pCamera->SetViewPort(0, 0, cx, cy);
	g_pCamera->SetFrustum( 30.0f, float(cx)/float(cy), 0.1f, 100.0);
	g_pCamera->Update();
};

bool find_hit( float x, float y, Wml::Ray3f & ray, rms::Frame3f & vHitFrame ) 
{
	// find hit point
	const unsigned int * pViewport = g_pCamera->GetViewport();
	g_pCamera->GetPickRay( x, y, 
		pViewport[2], pViewport[3], ray.Origin, ray.Direction );
	return g_mesh.FindHitFrame(ray, vHitFrame);
}


void glut_mouse(int button, int state, int x, int y)
{
	g_bAltButtonDown = ( glutGetModifiers() == GLUT_ACTIVE_ALT );
	g_bCtrlButtonDown = ( glutGetModifiers() == GLUT_ACTIVE_CTRL );
	g_vLastMousePos = Wml::Vector2f(x,y);

	g_bRightMouseDown = g_bMiddleMouseDown = g_bLeftMouseDown = false;
	
	if ( button == GLUT_RIGHT_BUTTON ) {
		g_bRightMouseDown = ( state == GLUT_DOWN ) ? true : false;
	} else if ( button == GLUT_LEFT_BUTTON ) {
		g_bLeftMouseDown = ( state == GLUT_DOWN ) ? true : false;
	} else if ( button == GLUT_MIDDLE_BUTTON ) {
		g_bMiddleMouseDown = ( state == GLUT_DOWN ) ? true : false;
	}

	g_inputMode = NONE;
	if ( g_bAltButtonDown && g_bRightMouseDown ) 
		g_inputMode = DOLLY;
	else if ( g_bAltButtonDown && g_bLeftMouseDown ) 
		g_inputMode = ORBIT;
	else if ( g_bAltButtonDown && g_bMiddleMouseDown ) 
		g_inputMode = PAN;
	else if ( g_bLeftMouseDown && g_bCtrlButtonDown )
		g_inputMode = SCALE_EXPMAP;
	else if ( g_bRightMouseDown && g_bCtrlButtonDown )
		g_inputMode = ROTATE_EXPMAP;
	else if ( g_bLeftMouseDown )
		g_inputMode = MOVE_EXPMAP_SEED;
}



void glut_mouse_motion(int x, int y)
{
	static const float fPanScale = 0.01f;
	static const float fZoomScale = 0.01f;
	static const float fOrbitScale = 0.01f;
	static const float fExpMapScale = 0.005f;

	Wml::Vector2f vCur(x,y);
	Wml::Vector2f vDelta(vCur - g_vLastMousePos);
	g_vLastMousePos = vCur;

	rms::Frame3f vHitFrame;
	Wml::Ray3f vRay;
	bool bHit = find_hit(x, y, vRay, vHitFrame);

	switch ( g_inputMode ) {
		case DOLLY:
			g_pCamera->DollyZoom((-vDelta.Y() + vDelta.X()) * fZoomScale);
			glutPostRedisplay();
			break;
		case PAN:
			g_pCamera->PanLateral(vDelta.X() * fPanScale);
			g_pCamera->PanVertical(vDelta.Y() * fPanScale);
			glutPostRedisplay();
			break;
		case ORBIT:
			g_pCamera->OrbitLateral(-vDelta.X() * fOrbitScale);
			g_pCamera->OrbitVertical(vDelta.Y() * fOrbitScale);
			glutPostRedisplay();
			break;

		case MOVE_EXPMAP_SEED:
			if ( bHit ) {
				g_mesh.MoveExpMap( vHitFrame );
			}
			glutPostRedisplay();
			break;

		case SCALE_EXPMAP:
			g_mesh.ScaleExpMap( 1.0f + (vDelta.X() - vDelta.Y())*fExpMapScale );
			glutPostRedisplay();
			break;

		case ROTATE_EXPMAP:
			g_mesh.RotateExpMap( -(vDelta.X() + vDelta.Y())*fExpMapScale );
			glutPostRedisplay();
			break;
	}

}



// keyboard event handler
void glut_keyboard (unsigned char key, int x, int y)
{
	static int info_banner = 1;

	switch (key) {

	case 'd':  case 'D':
		g_displayMode = (DISPLAY_MODE)((g_displayMode + 1) % LAST_MODE);
		glutPostRedisplay();
		break;

	case 'q':
	case 'Q':
		exit (0);
		break;
	};
};




void glut_idle (void)
{
};


void print_usage()
{
	std::cout << "expmapCL  v1.0   (C) Ryan Schmidt, 2009" << std::endl;
	std::cout << "Usage: "
		      << "expmapCL [filename] [nbrtype] [nbrsize1] [nbrsize2]" << std::endl
		      << "[nbrtype] = \'g\'   -->  [nbrsize1] = geodesic radius" << std::endl
		      << "[nbrtype] = \'k\'   -->  [nbrsize1] = number of nbrs" << std::endl
		      << "[nbrtype] = \'h\'   -->  [nbrsize1] = geo radius, [nbrsize2] = knbrs" << std::endl;
}


enum EXPMAP_NBR_MODE {
	MaxGeoDist,
	MaxK,
	Hybrid
};

int main(int argc, char ** argv)
{

	bool bInteractive = true;
	if ( bInteractive ) {
	
		//LoadNewMesh("sphere.obj");

    LoadNewMesh(argv[1]);

		/* Glut initializations */
		glutInit (&argc, argv);
		glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
		glutInitWindowPosition(5, 5);
		glutInitWindowSize(800, 600);

		/* Main window creation and setup */
		glutCreateWindow("Discrete Exponential Map Demo");
		glutDisplayFunc(glut_main_display);
		glutReshapeFunc(glut_main_reshape);
		glutKeyboardFunc(glut_keyboard);
		glutMouseFunc(glut_mouse);
		glutMotionFunc(glut_mouse_motion);
		glutIdleFunc(glut_idle);

		glut_initgl();

		glutMainLoop();

	} else {

		if ( argc < 3 ) {
			print_usage();
			exit(-1);
		}
	
		const char * pFilename = argv[1];

		EXPMAP_NBR_MODE eMode;
		float fMaxGeoDist = 0;
		int nMaxK = 0;

		int oh_noes_FAIL = 0;
	
		const char * pMode = argv[2];
		if ( pMode[0] == 'k' ) {
			eMode = MaxK;
			nMaxK = atoi( argv[3] );
			if (nMaxK == 0 )
				oh_noes_FAIL = 1;
		} else if ( pMode[0] == 'g' ) {
			eMode = MaxGeoDist;
			fMaxGeoDist = atof( argv[3] );
			if ( fMaxGeoDist == 0 )
				oh_noes_FAIL = 1;
		} else if ( pMode[0] == 'h' ) {
			eMode = Hybrid;
			fMaxGeoDist = atof( argv[3] );
			if ( fMaxGeoDist == 0 )
				oh_noes_FAIL = 1;
			nMaxK = atoi( argv[4] );
			if (nMaxK == 0 )
				oh_noes_FAIL = 1;
		} else {
			oh_noes_FAIL = 1;
		}
		
		if ( oh_noes_FAIL ) {
			print_usage();
			exit(-1);
		}

		/* initialize data structures */

		rms::VFTriangleMesh mesh;
		std::string err;
		bool bOK = mesh.ReadOBJ(pFilename,err);	
		if ( ! bOK ) {
			std::cerr << "[expmapCL] error: could not read OBJ file " << pFilename << std::endl;
			exit(-1);
		}

		bool bIsMesh = (mesh.GetTriangleCount() > 0);
		if ( bIsMesh )
			std::cerr << "[expmapCL] OBJ has triangles - using mesh path" << std::endl;
		else
			std::cerr << "[expmapCL] OBJ has no triangles - using points-only path" << std::endl;

		if ( eMode == MaxGeoDist )
			std::cerr << "[expmapCL] computing all vertex expmaps with geo-radius " << fMaxGeoDist << std::endl;
		else if (eMode == MaxK )
			std::cerr << "[expmapCL] computing all vertex expmaps with K-geo-nbrhood " << nMaxK << std::endl;
		else if (eMode == Hybrid)
			std::cerr << "[expmapCL] computing all vertex expmaps with geo-radius " << fMaxGeoDist << " and fallback K-geo-nbrhood " << nMaxK << std::endl;
		

		_RMSTUNE_start(1);
		rms::ExpMapGenerator expmapgen;
		//expmapgen.SetUseNeighbourNormalSmoothing(false);
		//expmapgen.SetUseUpwindAveraging(false);
		rms::IMeshBVTree bvTree;
		if ( bIsMesh  ) {
			bvTree.SetMesh(&mesh);
			expmapgen.SetSurface(&mesh, &bvTree);
		} else {
			expmapgen.SetSurface(&mesh, NULL);
		}
		
		// compute initial expmap to initialize neighbour lists
		Wml::Vector3f vVertex, vNormal;
		mesh.GetVertex(1, vVertex, &vNormal);
		rms::Frame3f vInitFrame(vVertex, vNormal);
		expmapgen.SetSurfaceDistances( vInitFrame, 0.0f, 1 );

		_RMSTUNE_end(1);
		std::cerr << "[expmapCL] neighbour precomputation: " << _RMSTUNE_time(1) << "s" << std::endl;
		std::cerr << "           (threshold is : " << expmapgen.GetNeighbourThreshold() << ")" << std::endl;

		std::ostream & output = std::cout;

		/* compute expmap at each vertex */

		_RMSTUNE_accum_init(2);
		
		std::vector<unsigned int> vIdx;
		std::vector<float> vU, vV;
		rms::VFTriangleMesh::vertex_iterator curv(mesh.BeginVertices()), endv(mesh.EndVertices());
		unsigned int nExpmaps = mesh.GetVertexCount();
		float fAvgNbrs = 0;
		while ( curv != endv ) {
			_RMSTUNE_start(2);
			rms::IMesh::VertexID vID = *curv;  curv++;
			mesh.GetVertex(vID, vVertex, &vNormal);
			rms::Frame3f vSeedFrame(vVertex, vNormal);

			switch ( eMode ) {
				case MaxK:
					expmapgen.SetSurfaceDistances( vSeedFrame, 0.0f, nMaxK);
					break;
				case MaxGeoDist:
					expmapgen.SetSurfaceDistances( vVertex, 0.0f, fMaxGeoDist, &vSeedFrame);
					break;
				case Hybrid:
					expmapgen.SetSurfaceDistances( vSeedFrame, 0.0f, fMaxGeoDist, nMaxK);
					break;
			}


			vIdx.resize(0); vU.resize(0); vV.resize(0);
			expmapgen.GetVertexUVs(vIdx, vU, vV);
			_RMSTUNE_end(2);
			_RMSTUNE_accum(2);
			size_t nCount = vIdx.size();

//			output << (vID+1) << " -999 -999" << std::endl;
			for ( unsigned int k = 0; k < nCount; ++k ) {
//				output << (vIdx[k]+1) << " " << vU[k] << " " << vV[k] << " " << sqrt(vU[k]*vU[k]+vV[k]*vV[k]) << std::endl;
//				output << (vIdx[k]+1) << " " << vU[k] << " " << vV[k] << std::endl;
				output << (vID+1) << " " << (vIdx[k]+1) << " " << vU[k] << " " << vV[k] << std::endl;
			}

			fAvgNbrs += (float)nCount / (float)nExpmaps;
		}
		float fTotalTime = _RMSTUNE_accum_time(2);
		float EPS = nExpmaps / fTotalTime;
		std::cerr << "[expmapCL] computed " << nExpmaps << " expmaps in "  << fTotalTime << "s" << std::endl;
		std::cerr << "    average nbr count: " << fAvgNbrs << std::endl;
		std::cerr << "    expmaps per second " << EPS << std::endl;

	}

	//getchar();

	return 0;
}

