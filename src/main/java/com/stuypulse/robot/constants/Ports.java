/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public interface Drivetrain {
        // Motors
        int LEFT_TOP = 10;
        int LEFT_MIDDLE = 11;
        int LEFT_BOTTOM = 12;

        int RIGHT_TOP = 13;
        int RIGHT_MIDDLE = 14;
        int RIGHT_BOTTOM = 15;

        // Soleniods
        int GEAR_SHIFT_FORWARD = 0;
        int GEAR_SHIFT_REVERSE = 1;
    }

    interface Grayhill {
        // Sensors
        int LEFT_A = 0;
        int LEFT_B = 1;

        int RIGHT_A = 2;
        int RIGHT_B = 3;
    }
}
