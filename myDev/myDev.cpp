// myDev.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"

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

// mine head
#include "DeltaControl.h"
#pragma comment(lib, "DeltaUSB580.lib")

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
cDirectionalLight *light;

// width and height of the current window display
int displayW = 0;
int displayH = 0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevice* hapticDevice;

// a label to display the haptic device model
cLabel* labelHapticDeviceModel;

// a label to display the position [mm] of the haptic device
cLabel* labelHapticDevicePosition;

// a global variable to store the position [m] of the haptic device
cVector3d hapticDevicePosition;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// a small sphere (cursor) representing the haptic device 
cShapeSphere* cursor;

// a line representing the velocity vector of the haptic device
cShapeLine* velocity;

// use damping (ON/OFF)
bool useDamping = false;

// use force field (ON/OFF)
bool useForceField = true;

// indicates if the haptic simulation currently running
bool simulationRunning = false;

// indicates if the haptic simulation has terminated
bool simulationFinished = false;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// my dev control and varibles
CDeltaUSBControl deltaCtrl;

double dF[3] ={0};
double dPos[3] = {0};
double dRad[6] = {0};
BYTE  statu;
double dT[3]={0};
int		nErrCode=0;
//bool blOutRange = FALSE;



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

//int _tmain(int argc, _TCHAR* argv[])		// 自动生成的
int main(int argc, char* argv[])
{

	//-----------------------------------------------------------------------
	// INITIALIZATION
	//-----------------------------------------------------------------------

	printf ("\n");
	printf ("-----------------------------------\n");
	printf ("CHAI3D\n");
	printf ("Demo: 01-mydevice\n");
	printf ("Copyright 2003-2012\n");
	printf ("-----------------------------------\n");
	printf ("\n\n");
	printf ("Keyboard Options:\n\n");
	printf ("[1] - Enable/Disable potential field\n");
	printf ("[2] - Enable/Disable damping\n");
	printf ("[x] - Exit application\n");
	printf ("\n\n>\r");


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
	camera->set( cVector3d (0.5, 0.0, 0.0),    // camera position (eye)
		cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
		cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector
	// my camera
	/*camera->set( cVector3d (0.0, 0.0,-0.5),    // camera position (eye)
		cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
		cVector3d (-1.0, 0.0, 0.0));   // direction of the (up) vector*/

	// set the near and far clipping planes of the camera
	// anything in front/behind these clipping planes will not be rendered
	camera->setClippingPlanes(0.01, 10.0);

	// create a light source
	light = new cDirectionalLight(world);

	// add light to world
	world->addChild(light);

	// enable light source
	light->setEnabled(true);                   

	// define the direction of the light beam
	light->setDir(-1.0, 0.0, 0.0); 

	// create a sphere (cursor) to represent the haptic device
	cursor = new cShapeSphere(0.01);

	// add cursor to the world
	world->addChild(cursor);

	// create a small line to illustrate the velocity of the haptic device
	velocity = new cShapeLine(cVector3d(0,0,0), 
		cVector3d(0,0,0));

	// add line to the world
	world->addChild(velocity);


	//-----------------------------------------------------------------------
	// HAPTIC DEVICE
	//-----------------------------------------------------------------------
	
	
	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// get a handle to the first haptic device
	handler->getDevice(hapticDevice, 0);

	// open a connection with the haptic device
	hapticDevice->open();

	// retrieve information about the current haptic device
	cHapticDeviceInfo info = hapticDevice->getSpecifications();

	// if the haptic device provides orientation sensing (stylus), 
	// a reference frame is displayed
	if (info.m_sensedRotation == true)
	{
		// display a reference frame
		cursor->setShowFrame(true);

		// set the size of the reference frame
		cursor->setFrameSize(0.05, 0.05);
	}

	// if the device has a gripper, enable the gripper to behave like a user switch
	hapticDevice->setEnableGripperUserSwitch(true);
	

	// my device open
	deltaCtrl.ConnectDevice();


	//-----------------------------------------------------------------------
	// WIDGETS
	//-----------------------------------------------------------------------

	// create a font
	cFont *font = NEW_CFONTCALIBRI20();

	// create a label to display the haptic device model
	labelHapticDeviceModel = new cLabel(font);
	camera->m_frontLayer->addChild(labelHapticDeviceModel);
	labelHapticDeviceModel->setString("haptic device: "+info.m_modelName);

	// create a label to display the position of haptic device
	labelHapticDevicePosition = new cLabel(font);
	camera->m_frontLayer->addChild(labelHapticDevicePosition);

	// create a label to display the haptic rate of the simulation
	labelHapticRate = new cLabel(font);
	//camera->m_frontLayer->addChild(labelHapticRate);	//	mine rate


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
	return 0;
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
	// escape key
	if ((key == 27) || (key == 'x'))
	{
		close();
		exit(0);
	}

	// option 1:
	if (key == '1')
	{
		useForceField = !useForceField;
		if (useForceField)
			printf ("> Enable force field     \r");    
		else
			printf ("> Disable force field    \r");    
	}

	// option 2:
	if (key == '2')
	{
		useDamping = !useDamping;
		if (useDamping)
			printf ("> Enable damping         \r");    
		else
			printf ("> Disable damping        \r");   
	}
}

//---------------------------------------------------------------------------

void menuSelect(int value)
{
	static int _wx, _wy, _ww, _wh;

	switch (value)
	{
		// fullscreen display
	case OPTION_FULLSCREEN:
		_wx = glutGet(GLUT_WINDOW_X);
		_wy = glutGet(GLUT_WINDOW_Y);
		_ww = glutGet(GLUT_WINDOW_WIDTH);
		_wh = glutGet(GLUT_WINDOW_HEIGHT);
		glutFullScreen();
		break;

		// window display
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
	hapticDevice->close();

	// my device close
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

	// update position of label
	labelHapticDeviceModel->setLocalPos(10, displayH - 30, 0.0);

	// update position of label and content
	double posX = 1000 * hapticDevicePosition.x();
	double posY = 1000 * hapticDevicePosition.y();
	double posZ = 1000 * hapticDevicePosition.z();

	labelHapticDevicePosition->setString("position: " + cStr(posX, 0) + " " +
		cStr(posY, 0) + " " +
		cStr(posZ, 0));

	labelHapticDevicePosition->setLocalPos(10, displayH - 50, 0.0);

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
	// initialize frequency counter
	frequencyCounter.reset();

	// main haptic simulation loop
	while(simulationRunning)
	{
		/////////////////////////////////////////////////////////////////////
		// READ HAPTIC DEVICE
		/////////////////////////////////////////////////////////////////////

		// my reading position
		// bool SetFGetPosStatues(double dF[3],double dPos[3], BYTE & statu,double dT[3],bool & blOutRange);
		bool blIO = false;
		// blIO = deltaCtrl.SetFGetPosStatues(dF,dPos,statu,dT, blOutRange);
		blIO = deltaCtrl.SetFTGetPosStatus(dF,dT,nErrCode);
		deltaCtrl.GetPos(dPos);
		deltaCtrl.GetRad(dRad);
		deltaCtrl.GetState(statu);
		

		// read position 
		cVector3d position;

		hapticDevice->getPosition(position);

		// read orientation 
		cMatrix3d rotation;
		hapticDevice->getRotation(rotation);

		// read gripper position
		double gripperAngle;
		hapticDevice->getGripperAngleRad(gripperAngle);

		// read linear velocity 
		cVector3d linearVelocity;
		hapticDevice->getLinearVelocity(linearVelocity);

		// read angular velocity
		cVector3d angularVelocity;
		hapticDevice->getAngularVelocity(angularVelocity);

		// read gripper angular velocity
		double gripperAngularVelocity;
		hapticDevice->getGripperAngularVelocity(gripperAngularVelocity);

		// read userswitch status (button 0)
		bool button0, button1, button2, button3;
		button0 = false;
		button1 = false;
		button2 = false;
		button3 = false;

		hapticDevice->getUserSwitch(0, button0);
		hapticDevice->getUserSwitch(1, button1);
		hapticDevice->getUserSwitch(2, button2);
		hapticDevice->getUserSwitch(3, button3);

		/////////////////////////////////////////////////////////////////////
		// UPDATE 3D MODELS
		/////////////////////////////////////////////////////////////////////

		// update arrow
		velocity->m_pointA = position;
		velocity->m_pointB = cAdd(position, linearVelocity);

		// update position and orientation of cursor
		// change position for mine***********************
		for(int i=0;i<3;i++)
		{
			position[i] = dPos[i];		// update potion
		}
		cursor->setLocalPos(position);
		cursor->setLocalRot(rotation);

		// adjust the  color of the cursor according to the status of
		// the user switch (ON = TRUE / OFF = FALSE)
		if (button0)
		{
			cursor->m_material->setGreenMediumAquamarine(); 
		}
		else if (button1)
		{
			cursor->m_material->setYellowGold();
		}
		else if (button2)
		{
			cursor->m_material->setOrangeCoral();
		}
		else if (button3)
		{
			cursor->m_material->setPurpleLavender();
		}
		else
		{
			cursor->m_material->setBlueRoyal();
		}

		// update global variable for graphic display update
		hapticDevicePosition = position;


		/////////////////////////////////////////////////////////////////////
		// COMPUTE AND SEND FORCE AND TORQUE
		/////////////////////////////////////////////////////////////////////

		cVector3d force (0,0,0);
		cVector3d torque (0,0,0);
		double gripperForce = 0.0;

		// apply force field
		if (useForceField)
		{
			// compute linear force
			double Kp = 200; // [N/m]
			cVector3d forceField = -Kp * position;

			// my modification**********************
			cVector3d positionT;
			positionT = position;
			positionT.y(0);
			positionT.x(0);
			forceField = -50*positionT;
			// endpoint*********************

			// deltaControl force test******************
			if (0)
			{
				if (dPos[2]> -0.12)
				{
					dF[2] = -1000*(dPos[2]+0.12);
					//dF[2] = -8;
				}
				else
					dF[2] =0;
			}			
			// deltaEnd*****************
			force.add(forceField);

			// compute angular torque
			double Kr = 0.05; // [N/m.rad]
			cVector3d axis;
			double angle;
			rotation.toAngleAxis(angle, axis);
			torque = (-Kr * angle) * axis;
		}

		// apply damping term
		if (useDamping)
		{
			cHapticDeviceInfo info = hapticDevice->getSpecifications();

			// compute linear damping force
			double Kv = 1.0 * info.m_maxLinearDamping;
			cVector3d forceDamping = -Kv * linearVelocity;
			force.add(forceDamping);

			// compute angluar damping force
			double Kvr = 1.0 * info.m_maxAngularDamping;
			cVector3d torqueDamping = -Kvr * angularVelocity;
			torque.add(torqueDamping);

			// compute gripper angular damping force
			double Kvg = 1.0 * info.m_maxGripperAngularDamping;
			gripperForce = gripperForce - Kvg * gripperAngularVelocity;
		}

		// send computed force, torque and gripper force to haptic device	
		hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);

		// update frequency counter
		frequencyCounter.signal(1);
	}

	// exit haptics thread
	simulationFinished = true;
}
