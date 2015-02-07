/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.fairportfirst.frc2012.drive;

import com.fairportfirst.frc2012.ui.*;

import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Joystick;
/**
 * The supreme central class whence all other classes are called
 * @author brennan
 */
public class MainDrive 
{
    DriverStationLCD dslcd;
    DriveStick drivestick;
    DriveTrain drivetrain;
    ShooterStick shooterstick;
    /**
     * This method is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
        drivestick = new DriveStick(1/*put the real port number here*/);
        shooterstick = new ShooterStick(2/*put the real port number here*/);
        drivetrain = new DriveTrain(2, 3, 4, 5 /*put the real port numbers here*/);
        
    }
    /**
     * This method is called periodically when the robot is in autonomous
     */
    public void autonomousPeriodic()
    {
        //implement the camera tracking code in here before you shoot
        //prepare for the shot (line up the drive train)
        //shoot
    }
    /**
     * This method is called periodically when the robot is in teleop
     */
    public void teleopPeriodic()
    {
        //sets the left and right sides of the drive train and uses the
        //joystick's twist feature to make it pivot. It should be intuitive
        //ask Brennan if you don't understand
        drivetrain.set(drivestick.getX() + drivestick.getTwist(),
                       drivestick.getX() - drivestick.getTwist());
        //shooter code will go here. Make sure to add an implementation for the
        //shooter joystick
        //add automatic code here on the side to line up the robot to shoot
        //put wedge code here.
        updateDashboard();
    }
    /**
     * Updates the dashboard (derp)
     */
    public void updateDashboard()
    {
        dslcd.println(DriverStationLCD.Line.kMain6, 1,
        "Output to left side: " + drivestick.getX() + drivestick.getTwist());
        dslcd.println(DriverStationLCD.Line.kUser2, 1,
        "Output to right side: " + (drivestick.getX() - drivestick.getTwist()));
    }
    
}
