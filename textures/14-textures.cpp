//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2012, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.0.0 $Rev: 774 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif

// MINE***********************
#include "DeltaControl.h"
#pragma comment(lib, "DeltaUSB580.lib")
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

// initial size (width/height) in pixels of the window display
const int WINDOW_SIZE_W         = 800;
const int WINDOW_SIZE_H         = 800;

// mouse menu options (right button)
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;

// use stereo display
const bool USE_STEREO_DISPLAY   = false;


//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// width and height of the current window display
int displayW = 0;
int displayH = 0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevice* hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a few mesh objects
cMesh* object0;
cMesh* object1;
cMesh* object2;
cMesh* object3;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// indicates if the haptic simulation currently running
bool simulationRunning = false;

// indicates if the haptic simulation has terminated
bool simulationFinished = false;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// root resource path
string resourceRoot;

// MINE******************
// my dev control and varibles

CDeltaUSBControl deltaCtrl;
double dF[3] ={0};
double dPos[3] = {0};
double dRad[6] = {0};
BYTE  statu;
double dT[3]={0};
int		nErrCode=0;


//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a key from the representing is pressed
void keySelect(unsigned char key, int x, int y);

// callback when the right mouse button is pressed to select a menu item
void menuSelect(int value);

// function called before exiting the application
void close(void);

// main graphics timer and callback
void graphicsTimer(int data);
void updateGraphics(void);

// main haptics loop
void updateHaptics(void);


//===========================================================================
/*
    DEMO:    14-textures.cpp

    This example illustrates the use of haptic textures project on mesh
    surfaces.

    In the main haptics loop function  "updateHaptics()" , the position
    of the haptic device is retrieved at each simulation iteration.
    The interaction forces are then computed and sent to the haptic device.
*/
//===========================================================================

int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    printf ("\n");
    printf ("-----------------------------------\n");
    printf ("CHAI3D\n");
    printf ("Demo: 14-textures\n");
    printf ("Copyright 2003-2012\n");
    printf ("-----------------------------------\n");
    printf ("\n\n");
    printf ("Keyboard Options:\n\n");
    printf ("[x] - Exit application\n");
    printf ("\n\n>\r");

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


    //-----------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //-----------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and oriente the camera
    camera->set( cVector3d (0.0, 0.0, 1.0),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
                 cVector3d (0.0, 1.0, 0.0));   // direction of the (up) vector
	
	//MINE	frame direction*******************************
	/*camera->set( cVector3d (0.0, 0.0, -2),    // camera position (eye)
		cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
		cVector3d (-1.0, 0.0, 0.0));   // direction of the (up) vector*/

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // enable shadow casting
    camera->setUseShadowCasting(true);

    // create a light source
    light = new cSpotLight(world);

    // attach light to camera
    world->addChild(light);    

    // enable light source
    light->setEnabled(true);                   

    // position the light source
     light->setLocalPos( 0.0, 0.0, 0.7);             
	// light->setLocalPos( 0.0, 0.0, -3);             

    // define the direction of the light beam
    light->setDir(0.0, 0.0, -1.0);             
//	  light->setDir(0.0, 0.0, 1.0); 

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);       


    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // create a 3D tool and add it to the world
    tool = new cToolCursor(world);
    camera->addChild(tool);

    // position tool in respect to camera
    tool->setLocalPos(-1.0, 0.0, 0.0);
	//tool->setLocalPos(0,0,1.5);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // initialize tool by connecting to haptic device
    tool->start();

    // set radius of tool
    double toolRadius = 0.01;
	//double toolRadius = 0.1;

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

    // define a radius for the tool
    tool->setRadius(toolRadius);

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

	// my device open		**********************MINE
	deltaCtrl.ConnectDevice();


    //-----------------------------------------------------------------------
    // CREATING OBJECTS
    //-----------------------------------------------------------------------

    // properties
    double maxForce     = hapticDeviceInfo.m_maxLinearForce;
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
    double maxDamping   = hapticDeviceInfo.m_maxLinearDamping / workspaceScaleFactor;

	// *******************MINE high force and stiffness, LinearStiffnesss
	maxStiffness = 500;



    /////////////////////////////////////////////////////////////////////////
    // OBJECT 0:
    /////////////////////////////////////////////////////////////////////////

    // create a mesh
    object0 = new cMesh();

    // create plane
    cCreatePlane(object0, 0.3, 0.3);

    // create collision detector
    object0->createAABBCollisionDetector(toolRadius);

    // add object to world
    world->addChild(object0);

    // set the position of the object
    object0->setLocalPos(-0.2, -0.2, 0.0);

    // set graphic properties
    bool fileload;
    object0->m_texture = new cTexture2d();
    fileload = object0->m_texture->loadFromFile(RESOURCE_PATH("resources/images/sand.bmp"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = object0->m_texture->loadFromFile("../../../bin/resources/images/sand.bmp");
        #endif
    }
    if (!fileload)
    {
        printf("Error - Texture image failed to load correctly.\n");
        close();
        return (-1);
    }

    // enable texture mapping
    object0->setUseTexture(true);
    object0->m_material->setWhite();

    // set haptic properties
    object0->m_material->setStiffness(0.8 * maxStiffness);
    object0->m_material->setStaticFriction(0.3);
    object0->m_material->setDynamicFriction(0.2);
    object0->m_material->setTextureLevel(1.0);
    object0->m_material->setRenderTriangles(true, false);


    /////////////////////////////////////////////////////////////////////////
    // OBJECT 1:
    ////////////////////////////////////////////////////////////////////////

    // create a mesh
    object1 = new cMesh();

    // create plane
    cCreatePlane(object1, 0.3, 0.3);

    // create collision detector
    object1->createAABBCollisionDetector(toolRadius);

    // add object to world
    world->addChild(object1);

    // set the position of the object
    object1->setLocalPos(0.2, -0.2, 0.0);

    // set graphic properties
    object1->m_texture = new cTexture2d();
	fileload = object1->m_texture->loadFromFile(RESOURCE_PATH("resources/images/whitefoam.jpg"));
	if (!fileload)
	{
			#if defined(_MSVC)
			fileload = object1->m_texture->loadFromFile("../../../bin/resources/images/whitefoam.jpg");
			#endif
	}
	if (!fileload)
	{
			printf("Error - Texture image failed to load correctly.\n");
			close();
			return (-1);
	}

    // enable texture mapping
    object1->setUseTexture(true);
    object1->m_material->setWhite();

    // set haptic properties
    object1->m_material->setStiffness(0.1 * maxStiffness);
    object1->m_material->setStaticFriction(0.0);
    object1->m_material->setDynamicFriction(0.3);
    object1->m_material->setTextureLevel(1.5);
    object1->m_material->setRenderTriangles(true, false);


    /////////////////////////////////////////////////////////////////////////
    // OBJECT 2:
    /////////////////////////////////////////////////////////////////////////

    // create a mesh
    object2 = new cMesh();

    // create plane
    cCreatePlane(object2, 0.3, 0.3);

    // create collision detector
    object2->createAABBCollisionDetector(toolRadius);

    // add object to world
    world->addChild(object2);

    // set the position of the object
    object2->setLocalPos(0.2, 0.2, 0.0);

    // set graphic properties
    object2->m_texture = new cTexture2d();
    fileload = object2->m_texture->loadFromFile(RESOURCE_PATH("resources/images/brownboard.jpg"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = object2->m_texture->loadFromFile("../../../bin/resources/images/brownboard.jpg");
        #endif
    }
    if (!fileload)
    {
        printf("Error - Texture image failed to load correctly.\n");
        close();
        return (-1);
    }

    // enable texture mapping
    object2->setUseTexture(true);
    object2->m_material->setWhite();

    // set haptic properties
    object2->m_material->setStiffness(0.4 * maxStiffness);
    object2->m_material->setStaticFriction(0.2);
    object2->m_material->setDynamicFriction(0.2);
    object2->m_material->setTextureLevel(0.5);
    object2->m_material->setRenderTriangles(true, false);
    

    /////////////////////////////////////////////////////////////////////////
    // OBJECT 3:
    ////////////////////////////////////////////////////////////////////////

    // create a mesh
    object3 = new cMesh();
    
    // create plane
    cCreatePlane(object3, 0.3, 0.3);

    // create collision detector
    object3->createAABBCollisionDetector(toolRadius);

    // add object to world
    world->addChild(object3);

    // set the position of the object
    object3->setLocalPos(-0.2, 0.2, 0.0);

    // set graphic properties
    object3->m_texture = new cTexture2d();
    fileload = object3->m_texture->loadFromFile(RESOURCE_PATH("./resources/images/blackstone.jpg"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = object3->m_texture->loadFromFile("../../../bin/resources/images/blackstone.jpg");
        #endif
    }
    if (!fileload)
    {
        printf("Error - Texture image failed to load correctly.\n");
        close();
        return (-1);
    }

    // enable texture mapping
    object3->setUseTexture(true);
    object3->m_material->setWhite();

    // set haptic properties
    object3->m_material->setStiffness(0.7 * maxStiffness);
    object3->m_material->setStaticFriction(0.4);
    object3->m_material->setDynamicFriction(0.3);
    object3->m_material->setTextureLevel(0.6);
    object3->m_material->setRenderTriangles(true, false);
    

    //-----------------------------------------------------------------------
    // WIDGETS
    //-----------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticRate);

    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(0.3, 0.3, 0.3),
                                cColorf(0.2, 0.2, 0.2),
                                cColorf(0.1, 0.1, 0.1),
                                cColorf(0.0, 0.0, 0.0));


    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    // simulation in now running!
    simulationRunning = true;

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve the resolution of the computer display and estimate the position
    // of the GLUT window so that it is located at the center of the screen
    int screenW = glutGet(GLUT_SCREEN_WIDTH);
    int screenH = glutGet(GLUT_SCREEN_HEIGHT);
    int windowPosX = (screenW - WINDOW_SIZE_W) / 2;
    int windowPosY = (screenH - WINDOW_SIZE_H) / 2;

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(WINDOW_SIZE_W, WINDOW_SIZE_H);
    if (USE_STEREO_DISPLAY)
    {
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
        camera->setUseStereo(true);
    }
    else
    {
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
        camera->setUseStereo(false);
    }
    glutCreateWindow(argv[0]);
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI3D");

    // create a mouse menu (right button)
    glutCreateMenu(menuSelect);
    glutAddMenuEntry("full screen", OPTION_FULLSCREEN);
    glutAddMenuEntry("window display", OPTION_WINDOWDISPLAY);
    glutAttachMenu(GLUT_RIGHT_BUTTON);


    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // start the main graphics rendering loop
    glutTimerFunc(30, graphicsTimer, 0);
    glutMainLoop();

    // close everything
    close();

    // exit
    return (0);
}

//---------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    // update the size of the viewport
    displayW = w;
    displayH = h;
    glViewport(0, 0, displayW, displayH);
}

//---------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    if ((key == 27) || (key == 'x'))
    {
        // close everything
        close();

        // exit application
        exit(0);
    }
}

//---------------------------------------------------------------------------

void menuSelect(int value)
{
    static int _wx, _wy, _ww, _wh;

    switch (value)
    {
        // enable full screen display
        case OPTION_FULLSCREEN:
            _wx = glutGet(GLUT_WINDOW_X);
            _wy = glutGet(GLUT_WINDOW_Y);
            _ww = glutGet(GLUT_WINDOW_WIDTH);
            _wh = glutGet(GLUT_WINDOW_HEIGHT);
            glutFullScreen();
            break;

        // reshape window to original size
        case OPTION_WINDOWDISPLAY:
            glutPositionWindow(_wx, _wy);
            glutReshapeWindow(_ww, _wh);
            break;
    }
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();
}

//---------------------------------------------------------------------------

void graphicsTimer(int data)
{
    // inform the GLUT window to call updateGraphics again (next frame)
    if (simulationRunning)
    {
        glutPostRedisplay();
    }

    glutTimerFunc(30, graphicsTimer, 0);
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    int px;

    // update haptic rate label
    labelHapticRate->setString ("haptic rate: "+cStr(frequencyCounter.getFrequency(), 0) + " [Hz]");

    px = (int)(0.5 * (displayW - labelHapticRate->getWidth()));
    labelHapticRate->setLocalPos(px, 15);

    // render world
    camera->renderView(displayW, displayH);

    // swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // main haptic simulation loop
    while(simulationRunning)
    {
        // compute global reference frames for each object

		// MINE ***********************IO process	
		bool blIO = false;
		//blIO = deltaCtrl.SetFGetPosStatues(dF,dPos,statu,dT, blOutRange);				// changing point		
		blIO = deltaCtrl.SetFTGetPosStatus(dF,dT,nErrCode);
		deltaCtrl.GetPos(dPos);
		deltaCtrl.GetRad(dRad);
		deltaCtrl.GetState(statu);

				dPos[0]-=0.15;

        world->computeGlobalPositions(true); // ͨ���������תƽ�Ƽ�����Ӷ����λ��

        // update position and orientation of tool
		tool->updatePose();

		// update my pose************MINE***************** we get the force in
		tool->updateMyPose(dPos);


        // compute interaction forces
        tool->computeInteractionForces();

		// we need to get the force out to our device****************	MINE
		tool->getLastComputedForce(dF);

        // send forces to device
        tool->applyForces();

        // update frequency counter
        frequencyCounter.signal(1);
    }
    //tool->cGenericTool.m_deviceLocalPos
    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------
