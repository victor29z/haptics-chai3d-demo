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

// my library*******************
#include "DeltaControl.h"
#pragma comment(lib, "DeltaUSB580.lib")
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

// initial size (width/height) in pixels of the window display
const int WINDOW_SIZE_W         = 1200;
const int WINDOW_SIZE_H         = 800;

// mouse menu options (right button)
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;

// use stereo display
const bool USE_STEREO_DISPLAY   = false;

// my dev control and varibles*************

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
int displayW = 0;
int displayH = 0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevice* hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a few mesh objects
cMesh* palette;
cMesh* paper;

// copy of blank paper texture
cImage* paperOriginal;

// selected paint color
cColorb paintColor;

// a label to explain what is happenning
cLabel* labelMessage;

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
    DEMO:    15-paint.cpp

    This example illustrates the use of texture and image pixel manipulation.

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
    printf ("Demo: 15-paint\n");
    printf ("Copyright 2003-2012\n");
    printf ("-----------------------------------\n");
    printf ("\n\n");
    printf ("Keyboard Options:\n\n");
    printf ("[c] - Clear page\n");
    printf ("[s] - Save to file as 'myPicture.jpg'\n");
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
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and oriente the camera
    camera->set( cVector3d (3.0, 0.0, 0.0),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // set orthogograhic camera mode
    camera->setOrthographicView(1.3);

    // disable multi-pass rendering to handle transparent objects
    camera->setUseMultipassTransparency(true);

    // create a light source
    light = new cDirectionalLight(world);

    // add light to world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);                   

    // define the direction of the light beam
    light->setDir(-1.0, 0.0,-0.4);             


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
    tool->setWorkspaceRadius(1.0);

    // define a radius for the tool
    double toolRadius = 0.01;
	//double toolRadius = 0.1;

    // set tool radius
    tool->setRadius(toolRadius);

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();
	workspaceScaleFactor =1;		// i change *************

	// my device open		**********************MINE
	bool blOpen=false;
	blOpen=deltaCtrl.ConnectDevice();
    //-----------------------------------------------------------------------
    // CREATING OBJECTS
    //-----------------------------------------------------------------------

    // properties
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
    double maxDamping   = hapticDeviceInfo.m_maxLinearDamping / workspaceScaleFactor;
	
	maxDamping=500;
	maxStiffness=500;


    /////////////////////////////////////////////////////////////////////////
    // PALETTE: 
    /////////////////////////////////////////////////////////////////////////

    // create a mesh
    palette = new cMesh();

    // create a plane
    cCreatePlane(palette, 0.5, 0.5);

    // create collision detector
    palette->createAABBCollisionDetector(toolRadius);

    // add object to world
    world->addChild(palette);

    // set the position of the object
    palette->setLocalPos(-0.25, -0.3, 0.0);
    palette->rotateAboutGlobalAxisDeg(cVector3d(0,1,0), 90);

    // set graphic properties
    cTexture2d* texture = new cTexture2d();
    palette->setTexture(texture);

	bool fileload = palette->m_texture->loadFromFile(RESOURCE_PATH("resources/images/palette.jpg"));
	if (!fileload)
	{
			#if defined(_MSVC)
			fileload = palette->m_texture->loadFromFile("../../../bin/resources/images/palette.jpg");
			#endif
	}
	if (!fileload)
	{
			printf("Error - Texture image failed to load correctly.\n");
			close();
			return (-1);
	}

    // we disable lighting properties for palette
    palette->setUseMaterial(false);

    // convert palette image from RGB to RGBA
    palette->m_texture->m_image->convert(GL_RGBA);

    // we set the white color (0xff, 0xff, 0xff) of the palette image to transparent (0x00).
    palette->m_texture->m_image->setTransparentColor(0xff, 0xff, 0xff, 0x00);

    // enable Mipmaps.
    palette->m_texture->setUseMipmaps(true);
    
    // enable transparency for this object
    palette->setUseTransparency(true);

    // enable texture mapping
    palette->setUseTexture(true);

    // set haptic properties
    palette->m_material->setStiffness(0.5 * maxStiffness);   
    palette->m_material->setStaticFriction(0.2);
    palette->m_material->setDynamicFriction(0.2);

    // initialize a default color for tool brush
    paintColor.setGreenMediumAquamarine();
    tool->m_hapticPoint->m_sphereProxy->m_material->setColor(paintColor);


    /////////////////////////////////////////////////////////////////////////
    // PAPER:
    ////////////////////////////////////////////////////////////////////////

    // create a mesh
    paper = new cMesh();

    // create a plane
    cCreatePlane(paper, 0.5, 0.5);

    // create collision detector
    paper->createAABBCollisionDetector(toolRadius);

    // add object to world
    world->addChild(paper);

    // set the position of the object
    paper->setLocalPos(-0.25, 0.3, 0.0);
    paper->rotateAboutGlobalAxisRad(cVector3d(0,1,0), cDegToRad(90));

    // set graphic properties
    paper->m_texture = new cTexture2d();
	fileload = paper->m_texture->loadFromFile(RESOURCE_PATH("resources/images/paper.jpg"));
	if (!fileload)
	{
			#if defined(_MSVC)
			fileload = palette->m_texture->loadFromFile("../../../bin/resources/images/paper.jpg");
			#endif
	}
	if (!fileload)
	{
			printf("Error - Texture image failed to load correctly.\n");
			close();
			return (-1);
	}

    // create a copy of paper so that we can clear page when requested
    paperOriginal = paper->m_texture->m_image->copy();

    // we disable lighting properties for paper
    paper->setUseMaterial(false);

    // enable texture mapping
    paper->setUseTexture(true);

    // set haptic properties
    paper->m_material->setStiffness(0.5 * maxStiffness);   
    paper->m_material->setStaticFriction(0.20);
    paper->m_material->setDynamicFriction(0.15);
    paper->m_material->setRenderTriangles(true, false);

    
    //-----------------------------------------------------------------------
    // WIDGETS
    //-----------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    labelHapticRate->m_fontColor.setGrayLevel(0.4);
    camera->m_frontLayer->addChild(labelHapticRate);

    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.00, 1.00, 1.00),
                                cColorf(0.95, 0.95, 0.95),
                                cColorf(0.85, 0.85, 0.85),
                                cColorf(0.80, 0.80, 0.80));

    // create a font
    cFont *font2 = NEW_CFONTCALIBRI32();

    // create a small message
    labelMessage = new cLabel(font2);
    labelMessage->m_fontColor.setGrayLevel(0.4);
    labelMessage->setString("press 'c' to clear page");
    labelMessage->setLocalPos(200, 50);
    camera->m_frontLayer->addChild(labelMessage);


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

    if (key == 'c')
    {
        // copy original image of paper to texture
        paperOriginal->copyTo(paper->m_texture->m_image);

        // update texture
        paper->m_texture->markForUpdate();

        // update console message
        printf ("> Image has been erased.           \r");
    }

    if (key == 's')
    {
        // save current texture image to file
        paper->m_texture->m_image->saveToFile("myPicture.jpg");

        // update console message
        printf ("> Image has been saved to file.     \r");
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

    // update message label
    px = (int)(0.5 * (displayW - labelMessage->getWidth()));
    labelMessage->setLocalPos(px, 45);

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
    // reset clock
    cPrecisionClock clock;
    clock.reset();

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // SIMULATION TIME    
        /////////////////////////////////////////////////////////////////////

        // stop the simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = clock.getCurrentTimeSeconds();

        // restart the simulation clock
        clock.reset();
        clock.start();

        // update frequency counter
        frequencyCounter.signal(1);


        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////
		
		// my device loop**************
		bool blIO = false;
		//blIO = deltaCtrl.SetFGetPosStatues(dF,dPos,statu,dT, blOutRange);				// changing point		
		blIO = deltaCtrl.SetFTGetPosStatus(dF,dT,nErrCode);
		deltaCtrl.GetPos(dPos);
		deltaCtrl.GetRad(dRad);
		deltaCtrl.GetState(statu);
		//	dPos[2]+=0.1;
		dPos[0]-=0.15;


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

        // get interaction forces magnitude
        double force = tool->m_lastComputedGlobalForce.length();

        // send forces to device
        tool->applyForces();


        /////////////////////////////////////////////////////////////////////
        // INTERACTION WITH PALETTE
        /////////////////////////////////////////////////////////////////////

        if (tool->isInContact(palette))
        {
            cCollisionEvent* contact = tool->m_hapticPoint->getCollisionEvent(0);
            if (contact != NULL)
            {
                // retrieve contact information
                cVector3d point = contact->m_localPos;
                cTriangle* triangle = contact->m_triangle;

                // retrieve pixel information
                int px, py;
                triangle->getPixelFromLocalPoint(point, px, py);

                // retrieve color information at pixel
                palette->m_texture->m_image->getPixelColor(px, py, paintColor);

                // update color of tool
                tool->m_hapticPoint->m_sphereProxy->m_material->setColor(paintColor);
            }
        }


        /////////////////////////////////////////////////////////////////////
        // INTERACTION WITH PAPER
        /////////////////////////////////////////////////////////////////////

        if (tool->isInContact(paper))
        {
            cCollisionEvent* contact = tool->m_hapticPoint->getCollisionEvent(0);
            if (contact != NULL)
            {
                // retrieve contact information
                cVector3d point = contact->m_localPos;
                cTriangle* triangle = contact->m_triangle;
                double v01 = contact->m_trianglePosV01;
                double v02 = contact->m_trianglePosV02;

				// retrieve pixel information
                int px, py;
                triangle->getPixel(v01, v02, px, py);

                // paint color at tool position
                const double K_INK = 20;
                const double K_SIZE = 10;
                const int BRUSH_SIZE = 25;

                double size = cClamp((K_SIZE * force), 0.0, (double)(BRUSH_SIZE));
                for (int x=-BRUSH_SIZE; x<BRUSH_SIZE; x++)
                {
                    for (int y=-BRUSH_SIZE; y<BRUSH_SIZE; y++)
                    {                        
                        // compute new color percentage
                        double distance = sqrt((double)(x*x+y*y));
                        if (distance <= size)
                        {
                            // get color at location
                            cColorb color, newColor;
                            paper->m_texture->m_image->getPixelColor(px+x, py+y, color);
                            
                            // compute color factor based of pixel position and force interaction 
                            double factor = cClamp(K_INK * timeInterval * cClamp(force, 0.0, 10.0) * cClamp(1 - distance/size, 0.0, 1.0), 0.0, 1.0);

                            // compute new color
                            newColor.setR((1.0 - factor) * color.getR() + factor * paintColor.getR());
                            newColor.setG((1.0 - factor) * color.getG() + factor * paintColor.getG());
                            newColor.setB((1.0 - factor) * color.getB() + factor * paintColor.getB());

                            // assign new color to pixel
                            paper->m_texture->m_image->setPixelColor(px+x, py+y, newColor);
                        }              
                    }
                }

                // update texture
                paper->m_texture->markForUpdate();
            }
        }

        // update frequency counter
        //frequencyCounter.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------
