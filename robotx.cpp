#include "WPILib.h"
#include "DashboardDataSender.h"
#include "RobotMap.h"

class TM_2013_Robot : public IterativeRobot
{
	// Xbox controller
	Joystick *controller;
	
	// Robot drive system
	RobotDrive *tmRobotDrive;
	
	// Talon motors
	Talon *leftWheel; 	  // left drive wheel
	Talon *rightWheel;	  // right drive wheel
	Talon *shooterMotor;  // two shooter wheels
	Talon *lifterMotors;  // lifts the robot
	Talon *frisbeeFeeder; // jackscrew
	
	// Digital Inputs (limit switches)
	DigitalInput *feederUpLS;	// upper limit switch on feeder
	DigitalInput *feederDownLS; // lower limit switch on feeder
	DigitalInput *lifterLS;		// lifter limit switch
	DigitalInput *frisbeeLoaderDetect; // detects frisbee in loader
	DigitalInput *frisbeePocketDetect; // detects frisbee in pocket
	
	// Digital Outputs (spike relays)
	Relay *cameraLight;	  // Green LEDs
	Relay *frisbeeLoader; // white wheel that loads frisbees
	
	// Gyroscope
	Gyro *gyro; // Detects angle changes in robot (useful for turning and driving in a straight line)
	
	// Timers
	Timer *timerAuto;	// timer to use in autonomous mode
	Timer *timerButton; // timer to prevent button spamming
	
	// Robot statuses
	enum eFrisbeeStatus {POCKET_DETECT, LOADER_ONLY, NO_FRISBEE, JAMMED}; // custom values for frisbee statuses
	eFrisbeeStatus frisbeeStatus;
	
	// Driver Station objects
	DriverStation *ds;		  // the driver station
	DriverStationLCD *dsLCD;  // the driver station's text field
	DashboardDataSender *dds; // sends data to the robot
	
	// Miscellaneous variables
	int autonomousState; // keeps track of autonomous mode stages
	float strength;    	 // defines shooting speed
	float leftSpeed; 	 // value of the left analog stick's y-axis
	float rightSpeed; 	 // value of the right analog stick's y-axis
	
public:
	// Initialize robot variables here
	TM_2013_Robot(void) {
		// Initialize the Xbox Controller
		controller = new Joystick(CONTROLLER);
		
		// Initialize wheels
		leftWheel  = new Talon(LEFT_WHEEL);
		rightWheel = new Talon(RIGHT_WHEEL);
		
		// Initialize robot drive system with the two wheels
		tmRobotDrive = new RobotDrive(leftWheel,rightWheel);
		// Invert the right wheel motor
		tmRobotDrive->SetInvertedMotor(tmRobotDrive->kRearRightMotor,true);
		
		// Initalize the Talons
		shooterMotor  = new Talon(SHOOTER_MOTOR);
		lifterMotors  = new Talon(LIFTER_MOTORS);
		frisbeeFeeder = new Talon(FRISBEE_FEEDER);
		
		// Initialize the Digital Inputs
		feederUpLS	 = new DigitalInput(FEEDER_UP_LS);
		feederDownLS = new DigitalInput(FEEDER_DOWN_LS);
		lifterLS	 = new DigitalInput(LIFTER_LS);
		frisbeeLoaderDetect = new DigitalInput(FRISBEE_LOADER_DETECT);
		frisbeePocketDetect = new DigitalInput(FRISBEE_POCKET_DETECT);
		
		// Initalize the Digital Ouputs (spike relays)
		frisbeeLoader = new Relay(FRISBEE_LOADER);
		cameraLight	  = new Relay(CAMERA_LIGHT);
		
		// Initalize the gyro
		gyro = new Gyro(GYRO);
		
		// Initalize the timers
		timerAuto   = new Timer();
		timerButton = new Timer();
		
		// Initialize miscellaneous variables
		autonomousState = 0; // start at beginning (first stage) of autonomous mode by default
		strength = 1.0; // default at maximum shooting strength
	}

	/********************************** Essential Functions *************************************/
	// Drive the robot
	void drive(float leftSpeed, float rightSpeed) {
		tmRobotDrive->TankDrive(leftSpeed,rightSpeed); // move the wheels at their specified speeds
	}
	// Stop the robot
	void stopRobot() {
		drive(0.0,0.0); // stop the robot wheels
	}
	// Stop a motor
	void stopMotor(Talon* motor) {
		motor->Set(0.0); // stop the motor
	}
	// Activate or deactivate a relay
	void toggleRelay(Relay* relay, string state) {
		if (state == "on")
			relay->Set(Relay::kForward); // activate the relay
		else if (state == "off")
			relay->Set(Relay::kOff); // deactivate the relay
	}
	// Check if a limit switch is activated or not
	bool isActive(DigitalInput* limitSwitch){
		return limitSwitch->Get() == 0; // deactivated switches have a value of 1
	}
	// Check the frisbee status of the robot
	void index(eFrisbeeStatus &frisbeeStatus) {
		if (isActive(frisbeePocketDetect) and isActive(feederDownLS))
			frisbeeStatus = POCKET_DETECT; // if frisbee in pocket and feeder down
		else if (isActive(frisbeeLoaderDetect) and not isActive(frisbeePocketDetect))
			frisbeeStatus = LOADER_ONLY; // if frisbee in loader but not in pocket
		else if (not isActive(frisbeeLoaderDetect) and not isActive(frisbeePocketDetect))
			frisbeeStatus = NO_FRISBEE; // if no frisbee in the robot
		else if (isActive(frisbeePocketDetect) and not isActive(feederDownLS))
			frisbeeStatus = JAMMED; // if frisbee in pocket but feeder not down
	}
	
	/************************************ Commands ***********************************************/
	// Use the gyro to drive in a straight line for a specified period of time
	void driveStraight(float time, float speed = 1.0) {
		timerAuto->Reset(); // reset the autonomous timer (this function will only be used in autonomous mode)
		gyro->Reset(); // reset the gyro value
		while (timerAuto->Get() < time) { // while specified time has not passed...
			float angle = gyro->GetAngle(); // get the current angle of the robot
			// adjust the wheel speeds based on the robot's angle
			leftSpeed  = speed-angle/50.0;
			rightSpeed = speed+angle/50.0;
			// make sure the speeds are within the range [-1.0,1.0]
			if (leftSpeed < 0.0) // if leftSpeed is negative...
				leftSpeed = max(leftSpeed,-1.0); // make sure it's above -1.0
			if (leftSpeed > 0.0) // if leftSpeed is positive...
				leftSpeed = min(leftSpeed,1.0); // make sure it's below 1.0
			if (rightSpeed < 0.0) // if rightSpeed is negative...
				rightSpeed = max(rightSpeed,-1.0); // make sure it's above -1.0
			if (rightSpeed > 0.0) // if rightSpeed is positive...
				rightSpeed = min(rightSpeed,1.0); // make sure it's below 1.0
			drive(leftSpeed,rightSpeed); // drive the robot using the adjusted wheel speeds
		}
		stopRobot(); // when the specified time has passed, stop moving the robot
	}
	// Turn the robot 'angle' degrees to the left or right
	void turn(string direction, float angle, float twistSpeed = 1.0) {
		gyro->Reset();
		float currentAngle = gyro->GetAngle(); // the robot's current angle (initially 0.0)
		if (direction == "left") {
			while (currentAngle > -angle) { // while the angle hasn't decreased by 'angle'...
				drive(-twistSpeed,twistSpeed); // turn the robot left
				currentAngle = gyro->GetAngle(); // update the current angle
			}
		}
		else if (direction == "right") {
			while (currentAngle < angle) { // while the angle hasn't increased by 'angle'...
				drive(twistSpeed,-twistSpeed); // turn the robot right
				currentAngle = gyro->GetAngle(); // update the current angle
			}
		}
		stopRobot(); // stop the robot once the goal angle has been reached
	}
	// Move the lifter up and down
	void moveLifter(string direction) {
		if (direction == "up" and not isActive(lifterLS)) // if the lifter is not too low
			lifterMotors->Set(1.0); // move the lifter down (lifts the robot)
		else if (direction == "down")
			lifterMotors->Set(-1.0); // move the lifter up (lowers the robot)
	}
	// Move the frisbee feeder up and down
	void moveFeeder(string direction) {
		float feederSpeed = -1.0; // feeder spins backwards, so -1.0 moves it up
		stopRobot(); // make sure the robot isn't moving
		if (direction == "up" and not isActive(feederUpLS)) { // if the upper limit switch isn't pressed...
			frisbeeFeeder->Set(feederSpeed); // start moving the feeder up
			while (not isActive(feederUpLS)) // while the feeder isn't all the way up...
				continue; // keep moving it
			stopMotor(frisbeeFeeder); // once the feeder reaches the top, stop moving it
		}
		else if (direction == "down" and not isActive(feederDownLS)) { // if the lower limit switch isn't pressed...
			frisbeeFeeder->Set(-feederSpeed); // start moving the feeder down
			while (not isActive(feederDownLS)) // while the feeder isn't all the way down...
				continue; // keep moving it
			stopMotor(frisbeeFeeder); // once the feeder reaches the bottom, stop moving it
		}
	}
	// Load a frisbee into the pocket
	void loadFrisbee() {
		moveFeeder("down"); // move the feeder down
		index(frisbeeStatus); // update frisbee status
		if (frisbeeStatus == LOADER_ONLY and frisbeeStatus != JAMMED) { // if the frisbee is in loader and not jammed...
			// while the frisbee is in the loader and not in the pocket...
			while (isActive(frisbeeLoaderDetect) and not isActive(frisbeePocketDetect)) // while frisbee is in loader and not in pocket...
				toggleRelay(frisbeeLoader,"on"); // move the frisbee loader
		}
		toggleRelay(frisbeeLoader,"off"); // when the frisbee has been loaded, stop the loader
	}
	// shoot a frisbee
	void shootFrisbee(float strength = 1.0) { // default at max strength
		index(frisbeeStatus); // update frisbee status
		if (frisbeeStatus == POCKET_DETECT) { // if frisbee is in pocket...
			stopRobot(); // make sure the robot isn't moving
			shooterMotor->Set(strength); // start the shooter wheels
			Wait(1.0); // wait for 1.0 second
			moveFeeder("up"); // raise the frisbee to shooter wheels
			stopMotor(shooterMotor); // stop the shooter wheels
			moveFeeder("down"); // move the feeder back down
		}
	}
	/********************************** Init Routines *****************************************/	
	// Runs once when the robot is turned on
	void RobotInit(void) {
		gyro->Reset();
	}
	// Runs once when the robot is disabled
	void DisabledInit(void) {
		gyro->Reset();
	}
	// Runs once when autonomous mode is initialized
	void AutonomousInit(void) {
		gyro->Reset();
		autonomousState = 0;
		index(frisbeeStatus);
		timerAuto->Start();
		timerAuto->Reset();
	}
	// Runs once when teleop mode is initialized
	void TeleopInit(void) {
		gyro->Reset();
		timerButton->Start();
		timerButton->Reset();
		index(frisbeeStatus);
	}

	/********************************** Periodic Routines *************************************/
	// Runs while the robot is disabled (after running DisabledInit)
	void DisabledPeriodic(void) {
		dds->sendIOPortData(); // allow the dashboard to communicate with the robot
	}
	// Runs while the robot is in autonomous mode (after running AutonomousInit)
	void AutonomousPeriodic(void) {
		timerAuto->Reset(); // reset the autonomous timer
		strength = ds->GetAnalogIn(1)/5.0; // set shooter strength based on dashboard's analog input 1 value (labeled "Strength")
		switch (autonomousState) {
		case 0: // Startup
			toggleRelay(cameraLight,"on"); // turn on camera LEDs
			autonomousState = 1; // move to case 1
			break;
		case 1: // Move feeder down
			moveFeeder("down");
			if (frisbeeStatus != JAMMED) // if frisbee is not jammed...
				autonomousState = 2; // move to case 2
			else autonomousState = 5; // otherwise, move to case 5 (don't shoot the frisbee)
			break;
		case 2: // Load frisbee if necessary
			index(frisbeeStatus); // update frisbee status
			if (frisbeeStatus != JAMMED) { // if no frisbee not jammed...
				loadFrisbee(); // load the frisbee (if possible)
				autonomousState = 3; // move to case 3
			}
			else autonomousState = 5; // if frisbee is jammed, move to case 5
			break;
		case 3: // Shoot a frisbee
			index(frisbeeStatus); // update frisbee status
			if (frisbeeStatus == POCKET_DETECT) { // if frisbee in pocket
				shootFrisbee(strength); // shoot a frisbee
				autonomousState = 4; // move to case 4
			}
			else if (timerAuto->Get() > 2.0) // if the frisbee isn't in the pocket after 2.0 seconds...
				autonomousState = 5; // move to case 5
			break;
		case 4: // Check frisbee status and reload if possible
			index(frisbeeStatus); // update frisbee status
			if (frisbeeStatus == LOADER_ONLY) // if frisbee only in loader...
				autonomousState = 1; // move back to case 1 to shoot again
			else autonomousState = 5; // if out of frisbees, move to case 5
			break;
		case 5: // Turn off camera LEDs
			toggleRelay(cameraLight,"off");
			autonomousState = 6;
			break;
		default: // Autonomous tasks have been completed; do nothing until autonomous period ends
			break;
		}
		dds->sendIOPortData(); // keep sending data to the robot throughout autonomous mode
	}
	// Runs while the robot is teleop mode (after running TeleopInit)
	void TeleopPeriodic(void) {
		strength = ds->GetAnalogIn(1)/5.0; // set shooter strength based on dashboard's analog input 1 value (labeled "Strength")
		index(frisbeeStatus); // update frisbee status
		leftSpeed  = controller->GetRawAxis(LEFT_ANALOG_Y);
		rightSpeed = controller->GetRawAxis(RIGHT_ANALOG_Y);
		drive(leftSpeed,rightSpeed); // this function gets called over and over again
		// Turn on camera LEDs with DPAD right
		if (controller->GetRawAxis(DPAD_X) > 0 and timerButton->Get() > 0.2) { // only allow one button press every 0.2 second
			toggleRelay(cameraLight,"on");
			timerButton->Reset();
		}
		// Turn off camera LEDs with DPAD left
		if (controller->GetRawAxis(DPAD_X) < 0 and timerButton->Get() > 0.2) { // only allow one button press every 0.2 second
			toggleRelay(cameraLight,"off");
			timerButton->Reset();
		}
		// Load frisbee with right bumper
		if (controller->GetRawButton(RIGHT_BUMPER)) {
			loadFrisbee();
		}
		// Shoot frisbee with triggers
		if (controller->GetRawAxis(TRIGGERS) != 0.0) { // detect trigger "wobble"
			shootFrisbee();
		}
		// Move feeder up with B button
		if (controller->GetRawButton(B)) {
			moveFeeder("up");
		}
		// Move feeder down with X button
		if (controller->GetRawButton(X)) {
			moveFeeder("down");
		}
		// Move lifter down with A button
		if (controller->GetRawButton(A)) {
			moveLifter("down");
		}
		// Move lifter up with Y button
		if (controller->GetRawButton(Y)) {
			moveLifter("up");
		}
		dds->sendIOPortData(); // send data to the robot
	}
};

START_ROBOT_CLASS(TM_2013_Robot);
