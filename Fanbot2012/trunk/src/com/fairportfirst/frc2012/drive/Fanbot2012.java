/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.fairportfirst.frc2012.drive;

import edu.wpi.first.wpilibj.Dashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.camera.AxisCamera;
//import com.fairportfirst.frc2012.vision.OpticalInterface;
//import edu.wpi.first.wpilibj.camera.AxisCameraException;
//import edu.wpi.first.wpilibj.image.ColorImage;
//import edu.wpi.first.wpilibj.image.NIVisionException;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 * @author 578
 */
public class Fanbot2012 extends IterativeRobot
{
    int driveMode = 3;
    //CANJaguar mrJaguar[];
    //CANJaguar mrJaguar4;
    //CANJaguar mrJaguar5;
    Drive mrDriveTrain;
    DriverStationLCD dslcd;
    Joystick mrJoystick1;
    Joystick mrJoystick2;
    Shooter mrShooter;
//    AxisCamera camera;
//    OpticalInterface opInterface;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
        dslcd = DriverStationLCD.getInstance();
        dslcd.println(DriverStationLCD.Line.kMain6, 1, "instantiating Bot2011");

//        opInterface = new OpticalInterface();
        // If you are not using the method for testing purposes, please comment them out.
        mrDriveTrain = new Drive(2, 3, 4, 5); // Change the port numbers if necessary.
//        mrShooter = new Shooter(2,3);
        mrJoystick1 = new Joystick(1);
        mrJoystick2 = new Joystick(2);
    }
    
    /**
     * This function is called periodically during autonomous mode.
     */
    public void autonomousPeriodic()
    {
//        camera = AxisCamera.getInstance();
//         //OKAY FOR FINAL THING 2ND PORT; USE 192.168.0.90 IF USE SWITCH, ELSE IF BRIGE USE 10.5.78.11
//        camera.writeResolution(AxisCamera.ResolutionT.k320x240);
//        camera.writeBrightness(25);
    }
    
    /**
     * This function is called periodically during teleop mode.
     */
    public void teleopPeriodic()
    {            
        // If you are not using the method for testing purposes, please comment them out.
//        if(isEnabled())
//        {
//            try {
//
//                camera = AxisCamera.getInstance();
//                // OKAY FOR FINAL THING 2ND PORT; USE 192.168.0.90 IF USE SWITCH, ELSE IF BRIGE USE 10.5.78.11
//                camera.writeResolution(AxisCamera.ResolutionT.k320x240);
//                camera.writeBrightness(50);
//                if(camera.freshImage())
//                {
//                    ColorImage image = camera.getImage();
//                    opInterface.updateImageAlgo(image);
//                    image.free();
//                }
//
//                //shooterTest();
//            } catch (AxisCameraException ex) {
//                System.out.println(ex);
//                ex.printStackTrace();
//            } catch (NIVisionException ex) {
//                System.out.println(ex);
//                ex.printStackTrace();
//            }
//        }
        driveTest();
    }
    
//    private void shooterTest()
//    {
//        if(!mrJoystick.getButton(Joystick.ButtonType.kTrigger)) {
//            double JagXVal;
//            JagXVal = -mrJoystick.getY();
//            mrShooter.setEachJags(JagXVal, JagXVal);
//        } else
//            mrShooter.setBothJagsX(roundDirectionNum(mrJoystick.getX()));
//    }

    private void driveTest()
    {
        double left=0;
        double right=0;
        switch (driveMode)
        {
            case 1:left = roundDirectionNum(mrJoystick1.getY()-mrJoystick1.getX());
                   right =  roundDirectionNum(-mrJoystick1.getY()-mrJoystick1.getX());
            break;
            case 2:left = mrJoystick1.getY();
                   right = -(mrJoystick2.getY());
            break;
            case 3:left = roundDirectionNum(mrJoystick1.getY()-mrJoystick1.getTwist());
                   right =  roundDirectionNum(-mrJoystick1.getY()-mrJoystick1.getTwist());
            break;
        }
//        double left = roundDirectionNum(mrJoystick1.getY()-mrJoystick1.getTwist());
//        double right = roundDirectionNum(-mrJoystick1.getY()-mrJoystick1.getTwist());
        mrDriveTrain.set(left, right);
//        dslcd.println(DriverStationLCD.Line.kUser2, 1, "Joystick 1 Y: " + mrJoystick1.getY());
//        dslcd.println(DriverStationLCD.Line.kUser2, 1, "Joystick 1 X: " + mrJoystick1.getX());
        dslcd.println(DriverStationLCD.Line.kUser2, 1, "Joystick 1 twist: " + mrJoystick1.getTwist());
//        dslcd.println(DriverStationLCD.Line.kUser3, 1, "Joystick 2 Y: " + mrJoystick2.getY());
//        dslcd.println(DriverStationLCD.Line.kUser3, 1, "Joystick 2 X: " + mrJoystick2.getX());
        dslcd.println(DriverStationLCD.Line.kUser3, 1, "Joystick 2 twist: " + mrJoystick2.getTwist());
        dslcd.println(DriverStationLCD.Line.kUser4, 1, "Left set value: " + left);
        dslcd.println(DriverStationLCD.Line.kUser5, 1, "Right set value: " + right);
        dslcd.updateLCD();
        updateDashBoard();
    }

    /**
     * 
     */
    public void updateDashBoard()
    {
        Dashboard lowDashData = DriverStation.getInstance().getDashboardPackerLow();
        DriverStationLCD.getInstance().updateLCD();
    }

    /**
     * Will round the number off if it is greater than 1 or less than -1
     * @param num the input
     * @return the input value rounded from -1 to 1
     */
    public double roundDirectionNum(double num)
    {
        if(num>1)
            num = 1;
        if(num<-1)
            num = -1;
        if(num/.08<1)
            num = 0;
        return num;
    }
}
