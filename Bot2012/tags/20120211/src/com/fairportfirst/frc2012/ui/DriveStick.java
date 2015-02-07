/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.fairportfirst.frc2012.ui;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The class that manages the joystick responsible for driving the robot
 * @author brennan
 */
public class DriveStick extends Joystick
{
    /**
     * Simply calls the superclass's constructor with a variable for the port
     * @param portNumberHere now what do you think this variable is for??
     */
    public DriveStick(int portNumberHere)
    {
        super(portNumberHere);
    }
}
