/*
 * LHSVision.cpp
 *
 *  Created on: Jan 26, 2016
 *      Author: Michael Conard
 *
 *  THING TO CHECK:
 *  	-In Internet Explorer, roborio-4237-frc.local,
 *  	 confirm the camera names in Silverlight. These
 *  	 should be "cam0", "cam1", or "cam2".
 *
 *  	-These names can be adjusted in the CAMERA
 *  	 namespace.
 */

#include "WPILib.h"
#include "LHSVision.h"

namespace XBOX		//Joystick Constants
{
	const int NODE_ID = 1;

	const int XBOX_A_BUTTON  = 1;
	const int XBOX_X_BUTTON = 3;

	const int XBOX_LEFT_X = 0;
	const int XBOX_LEFT_Y = 1;
}

namespace CAMERA	//Camera Names
{
	const char* CAM_ONE = "cam0";
	const char* CAM_TWO = "cam2";
}

//NOTE: Camera 1 is the default camera. Can toggle to 2 later.
LHSVision::LHSVision(RobotDrive* pRobot, Joystick* pXbox) //Constructor
{
	printf("File %18s Date %s Time %s Object %p\n",__FILE__,__DATE__, __TIME__, this);

	frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
	IMAQdxOpenCamera(CAMERA::CAM_ONE, IMAQdxCameraControlModeController, &session);
	Wait(.5);
	IMAQdxConfigureGrab(session);
	Wait(.5);
	IMAQdxStartAcquisition(session);
	Wait(.5);

	frame2 = imaqCreateImage(IMAQ_IMAGE_RGB, 0);

	mRobot = pRobot;
	mXbox = pXbox;
}

LHSVision::~LHSVision() //Destructor
{
	delete mXbox;
	delete mRobot;
}

//Used for closing and stopping acquisition of a camera
void LHSVision::StopCamera(int camNum)
{
	if(camNum == 1)	//Camera 1 Shutdown
	{
		IMAQdxStopAcquisition(session);
		Wait(.01);
		IMAQdxCloseCamera(session);
	}
	else			//Camera 2 Shutdown
	{
		IMAQdxStopAcquisition(session2);
		Wait(.01);
		IMAQdxCloseCamera(session2);
	}
}

//Used for opening, configuring, and starting acquisition of a camera
void LHSVision::StartCamera(int camNum)
{
	if(camNum == 1) //Camera 1 Startup
	{
		IMAQdxOpenCamera(CAMERA::CAM_ONE, IMAQdxCameraControlModeController, &session);
		Wait(.01);
		IMAQdxConfigureGrab(session);
		Wait(.01);
		IMAQdxStartAcquisition(session);
		Wait(.01);
	}
	else			//Camera 2 Startup
	{
		IMAQdxOpenCamera(CAMERA::CAM_TWO, IMAQdxCameraControlModeController, &session2);
		Wait(.01);
		IMAQdxConfigureGrab(session2);
		Wait(.01);
		IMAQdxStartAcquisition(session2);
		Wait(.01);
	}
}

//Send image to FRC PC Dashboard
void LHSVision::SendToDashboard(Image *image)
{
	CameraServer::GetInstance()->SetImage(image);
}

void LHSVision::UpdateVision() //by Michael
{
	if(mXbox->GetRawButton(XBOX::XBOX_A_BUTTON) == true)	//Toggle on A button press
	{
		while(mXbox->GetRawButton(XBOX::XBOX_A_BUTTON) == true)	//Prevents multi-press
		{}
		if(send == 1)
		{
			StopCamera(1);	//Stop Cam 1
			StartCamera(2);	//Start Cam 2
			printf("\nActivating Camera 2 - Raw Image Display\n\n");
			send = 2;
		}
		else
		{
			StopCamera(2);	//Stop Cam 2
			StartCamera(1);	//Start Cam 1
			printf("\nActivating Camera 1 - Raw Image Display\n\n");
			send = 1;
		}
	}

	if(send == 1)
	{
		IMAQdxGrab(session, frame, true, NULL);
		SendToDashboard(frame);		//Send Cam 1
		Wait(.1);
	}
	else
	{
		IMAQdxGrab(session2, frame2, true, NULL);
		SendToDashboard(frame2);	//Send Cam 2
		Wait(.1);
	}
}
