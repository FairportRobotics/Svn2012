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
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.*;
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
public class Dashbot2012 extends IterativeRobot
{
    public final int THREE_AXIS_PORT = 1;
    public final int LEFT_JOYSTICK_PORT = 2;
    public final int RIGHT_JOYSTICK_PORT = 3;
    public final int GAMEPAD_PORT = 4;

    //CANJaguar mrJaguar[];
    //CANJaguar mrJaguar4;
    //CANJaguar mrJaguar5;
    Drive mrDriveTrain;
    DriverStationLCD dslcd;
    DriveSystem ds;
    //Joystick mrJoystick1;
    //Joystick mrJoystick2;
    SendableChooser controlChooser;
    Shooter mrShooter;
//    AxisCamera camera;
//    OpticalInterface opInterface;
    SmartDashboard mrSmart;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
        NetworkTable.initialize();
        //mrSmart = new SmartDashboard();
        dslcd = DriverStationLCD.getInstance();
        dslcd.println(DriverStationLCD.Line.kMain6, 1, "instantiating Bot2011");

//        opInterface = new OpticalInterface();
        // If you are not using the method for testing purposes, please comment them out.
        mrDriveTrain = new Drive(2, 3, 4, 5); // Change the port numbers if necessary.
//        mrShooter = new Shooter(2,3);
        //mrJoystick1 = new Joystick(1);
        
        //mrSmart;
        //mrJoystick2 = new Joystick(2);

        //adds the drive options to the sendable chooser, with TankDrive as the
        //default
        controlChooser = new SendableChooser();
        controlChooser.addDefault("Default (Tank) Drive", new DriveSystem(
                DriveSystem.TANK, LEFT_JOYSTICK_PORT, RIGHT_JOYSTICK_PORT));
        controlChooser.addObject("X-Y Drive", new DriveSystem(
                DriveSystem.XYSINGLE, THREE_AXIS_PORT));
        controlChooser.addObject("Twist Drive", new DriveSystem(
                DriveSystem.TWIST, THREE_AXIS_PORT));
        controlChooser.addObject("Gamepad Drive", new DriveSystem(
                DriveSystem.GAMEPAD, GAMEPAD_PORT));
        SmartDashboard.putData("controlChooser", controlChooser);
    }
    /**
     * This class manages the joysticks and determines an output for the left
     * and the right regardless of the type of command selected.
     */
    public class DriveSystem
    {
        public static final int TANK = 1;
        public static final int XYSINGLE = 2;
        public static final int TWIST = 3;
        public static final int GAMEPAD = 4;
        private int driveType;
        private Joystick joystick1;//acts as the left joystick in tank drive
        private Joystick joystick2;//acts as the right joystick in tank drive
        /**
         * The constructor for a single joystick command type
         * @param type - use the static constant fields
         * @param joyport take a wild guess on what this is
         */
        public DriveSystem(int type, int joyport)
        {
            driveType = type;
            joystick1 = new Joystick(joyport);
        }
        /**
         * The constructor for a double joystick command type
         * @param type use the static constant fields
         * @param joyport1 the port for the left joystick
         * @param joyport2 the port for the right joystick
         */
        public DriveSystem(int type, int joyport1, int joyport2)
        {
            this(type, joyport1);
            joystick2 = new Joystick(joyport2);
        }
        /**
         * Returns the left output
         * @return
         */
        //1-leftx,2-lefty,4-rightx,5-righty,6-dpadl/r
        public double getLeft()
        {
            switch(driveType)
            {
                case 2://XY
                    return roundDirectionNum(joystick1.getY()-joystick1.getX());
                case 3://Twist
                    return roundDirectionNum(joystick1.getY() - joystick1.getTwist());
                case 4://gamepad tank
                    return joystick1.getRawAxis(2);
                case 5://gamepad split
                    return roundDirectionNum(joystick1.getRawAxis(2) - joystick1.getRawAxis(4));
                default://tank
                    return joystick1.getY();
            }
        }
        /**
         * Returns the right output
         * @return
         */
        public double getRight()
        {
            //The right side must be negated because hardware
            switch(driveType)
            {
                case 2://xy
                    return roundDirectionNum(-joystick1.getY() - joystick1.getX());
                case 3://twist
                    return roundDirectionNum(-joystick1.getY() - joystick1.getTwist());
                case 4://gamepad tank
                    return -joystick1.getRawAxis(5);
                case 5://gamepad split
                    return roundDirectionNum(-joystick1.getRawAxis(2)- joystick1.getRawAxis(4));
                default:
                    return -joystick2.getY();

            }
        }
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
        double left = ((DriveSystem)controlChooser.getSelected()).getLeft();
        double right = ((DriveSystem)controlChooser.getSelected()).getRight();
        mrDriveTrain.set(left, right);
        //dslcd.println(DriverStationLCD.Line.kUser2, 1, "Joystick 1: " + mrJoystick1.getY());
        //dslcd.println(DriverStationLCD.Line.kUser3, 1, "Joystick 2: " + mrJoystick2.getY());
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
        //mrSmart.
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
        return num;
    }
}
