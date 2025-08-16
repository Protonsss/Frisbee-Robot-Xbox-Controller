#ifndef ROBOTMAP_H
#define ROBOTMAP_H

// Talons
const int LEFT_WHEEL  	 = 1;
const int RIGHT_WHEEL 	 = 2;
const int SHOOTER_MOTOR  = 3;
const int LIFTER_MOTORS  = 4;
const int FRISBEE_FEEDER = 5;

//Digital Inputs
const int FEEDER_UP_LS 			 = 1;
const int FEEDER_DOWN_LS		 = 2;
const int LIFTER_LS				 = 5;
const int FRISBEE_LOADER_DETECT  = 7;
const int FRISBEE_POCKET_DETECT  = 8;

//Digital Outputs
const int FRISBEE_LOADER = 3;
const int CAMERA_LIGHT	 = 2;

//Gyro
const int GYRO = 1;

//Xbox Controller
const int CONTROLLER = 1;

// Axis Mapping
const int LEFT_ANALOG_X  = 1;
const int LEFT_ANALOG_Y  = 5;
const int TRIGGERS 		 = 3; // Left > 0 ; Right < 0
const int RIGHT_ANALOG_X = 4;
const int RIGHT_ANALOG_Y = 2;
const int DPAD_X		 = 6;
const int DPAD_Y		 = 7; // **doesn't seem work**

//	 Button Mapping
const int A					 = 1;
const int B 				 = 2;
const int X 				 = 3;
const int Y 				 = 4;
const int LEFT_BUMPER  		 = 5;
const int RIGHT_BUMPER 		 = 6;
const int BACK 				 = 7;
const int HOME				 = 8;
const int LEFT_ANALOG_PRESS  = 9;
const int RIGHT_ANALOG_PRESS = 10;

#endif
