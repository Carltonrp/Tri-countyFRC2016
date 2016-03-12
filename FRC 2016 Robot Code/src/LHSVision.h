/*
 * LHSVision.h
 *
 *  Created on: Jan 25, 2016
 *      Author: Michael Conard
 */

#ifndef LHS_VISION_H
#define LHS_VISION_H

class RobotDrive;
class Joystick;

class LHSVision
{
public:
	LHSVision(RobotDrive*, Joystick*); //Constructor
	~LHSVision();	//Destructor
	void SendToDashboard(Image*); //Send Image to Dashboard
	void UpdateVision();	//Toggle and Display Camera
	void StopCamera(int);	//Close Specified Camera
	void StartCamera(int);	//Start Specified Camera

private:
	IMAQdxSession session;
	Image* frame;

	IMAQdxSession session2;
	Image* frame2;

	RobotDrive* mRobot;
	Joystick* mXbox;

	int send = 1;
};

#endif
