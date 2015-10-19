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
    \version   3.0.0 $Rev: 750 $
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

// my library*******************
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

// my dev control and varibles

CDeltaUSBControl deltaCtrl;
double dF[3] ={0};
double dPos[3] = {0};
double dRad[6] = {0};
BYTE  statu;
double dT[3]={0};
int		nErrCode=0;

//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// width and height of the current window display
int displayW  = 0;
int displayH = 0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevice* hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a few spherical objects
cShapeSphere* sphere0;
cShapeSphere* sphere1;
cShapeLine* line;
cShapeCylinder* cylinder;

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
    DEMO:    04-shapes.cpp

    In this example we illustrate how to create some simple shape
	primitives.

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
    printf ("Demo: 04-shapes\n");
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
    camera->set( cVector3d (3.0, 0.0, 0.0),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector 

/*	camera->set( cVector3d (0.0, 0.0, -3.0),    // camera position (eye)
		cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
		cVector3d (-1.0, 0.0, 0.0));   // direction of the (up) vector*/

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // enable multi-pass rendering to handle transparent objects
    camera->setUseMultipassTransparency(true);

    // create a light source
    light = new cDirectionalLight(world);

    // add light to world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);                   

    // define the direction of the light beam
    light->setDir(-1.0, 0.0, 0.0);             


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
    world->addChild(tool);

    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // initialize tool by connecting to haptic device
    tool->start();

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(2.0);

    // define a radius for the tool
    tool->setRadius(0.03);

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

	// my device open		**********************MINE
	deltaCtrl.ConnectDevice();
	//double R[3][3] = {0,0,-1,0,-1,0,-1,0,0};
	//deltaCtrl.SetRotation(R);	


    //-----------------------------------------------------------------------
    // CREATING OBJECTS
    //-----------------------------------------------------------------------

    // temp variable
    cGenericEffect* newEffect;

    // stiffness properties
    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
	double maxLinearForce = hapticDeviceInfo.m_maxLinearForce;
	
	// pull up stiffness
	maxStiffness =300;

    // create a sphere 0
	sphere0 = new cShapeSphere(0.1);
	world->addChild(sphere0);
	sphere0->setLocalPos(0.0,-0.7, 0.0);
	sphere0->m_material->setRedFireBrick();
    newEffect = new cEffectSurface(sphere0);
    sphere0->addEffect(newEffect);
	sphere0->m_material->setStiffness(0.4 * maxStiffness);

    // create a sphere 1
	sphere1 = new cShapeSphere(0.1);
	world->addChild(sphere1);
	sphere1->setLocalPos(0.0, 0.7, 0.0);
	sphere1->m_material->setRedFireBrick();
    newEffect = new cEffectSurface(sphere1);
    sphere1->addEffect(newEffect);
	sphere1->m_material->setStiffness(0.4 * maxStiffness);

    // create a line
	line = new cShapeLine(sphere0->getLocalPos(), sphere1->getLocalPos());
	world->addChild(line);
	line->m_material->setWhite();
    newEffect = new cEffectMagnet(line);
    line->addEffect(newEffect);
	line->m_material->setMagnetMaxDistance(0.05);
	line->m_material->setMagnetMaxForce(0.1 * maxLinearForce);
	line->m_material->setStiffness(0.1 * maxStiffness);

	// create a cylinder
	cylinder = new cShapeCylinder(0.25, 0.25, 0.2);
	world->addChild(cylinder);
	cylinder->setLocalPos(0.0, 0.0, 0.0);
	cylinder->rotateAboutGlobalAxisDeg(cVector3d(1.0, 0.0, 0.0), 90);
	cylinder->m_material->setBlueCornflower();
    newEffect = new cEffectSurface(cylinder);
    cylinder->addEffect(newEffect);
	cylinder->m_material->setStiffness(0.8 * maxStiffness);

    
    //-----------------------------------------------------------------------
    // WIDGETS
    //-----------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticRate);


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
	// my device close**************************MINE
	memset(dF,0,sizeof(dF));
	memset(dT,0,sizeof(dT));

	//deltaCtrl.SetFGetPosStatues(dF,dPos,statu,dT,blOutRange);	// clear force
	deltaCtrl.SetTorqueGetPosStatus(dT,nErrCode);	// clear the force
	deltaCtrl.ReleaseDev();
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
	// precion clock
	cPrecisionClock clock;
	int i = 0;

    // main haptic simulation loop
    while(simulationRunning)
    {
		bool blIO = false;
		//blIO = deltaCtrl.SetFGetPosStatues(dF,dPos,statu,dT, blOutRange);				// changing point		
		blIO = deltaCtrl.SetFTGetPosStatus(dF,dT,nErrCode);
		deltaCtrl.GetPos(dPos);
		deltaCtrl.GetRad(dRad);
		deltaCtrl.GetState(statu);

	//	dPos[2]+=0.1;
		dPos[0]-=0.1;

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

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

		// stop clock
		double time = clock.stop();
		
		// check if contact occured with cylinder
		if(tool->isInContact(cylinder))
		{
			// get force applied on the y axis
			double force = tool->m_lastComputedGlobalForce.y();

			// shift cylinderaccording to force
            const double K = 0.5;
			double pos = cylinder->getLocalPos().y();
			pos = cClamp(pos - K * time * force,-0.5, 0.6);
			cylinder->setLocalPos(0.0, pos, 0.0);
		}

		// restart clock
		clock.start(true);

        // update frequency counter
        frequencyCounter.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------
