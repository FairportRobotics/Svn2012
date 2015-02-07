/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.fairportfirst.frc2012.drive;

import com.fairportfirst.frc2012.structures.*;
import com.fairportfirst.frc2012.ui.*;

import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
/**
 * The supreme central class whence all other classes are called
 * @author brennan
 */
public class MainDrive 
{
    DriverStationLCD dslcd;
    DriveStick driveStick;
    DriveTrain driveTrain;
    ShooterStick shooterStick;
    Lift mrLift;
    Shooter mrShooter;
    Wedge mrWedge;
    BallPickup ballPickup;

    SendableChooser controlChooser;
    Joystick mrJoystick1;
    Joystick mrJoystick2;
    Joystick mrJoystick3;
    private static int TANK_DRIVE = 1;
    private static int TILT_DRIVE = 2;
    private static int TWIST_DRIVE = 3;
    DriveType driveType;
    private double left = 0;
    private double right = 0;

    
    /**
     * This method is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
        NetworkTable.initialize();
        driveType = new DriveType(TANK_DRIVE);
        driveStick = new DriveStick(1/*put the real port number here*/);
        shooterStick = new ShooterStick(2/*put the real port number here*/);
        driveTrain = new DriveTrain(2, 3, 4, 5 /*put the real port numbers here*/);
        mrShooter = new Shooter(6, 7, 8); // CAN Devices 6 and 7 for wheels, 8 for turret.
        mrWedge = new Wedge(1); // PWM Slot 1
        mrLift = new Lift(2); // PWM Slot 2
        ballPickup = new BallPickup(3);

        mrJoystick1 = new Joystick(1);
        mrJoystick2 = new Joystick(2);
        mrJoystick3 = new Joystick(3);


        dslcd = DriverStationLCD.getInstance();
        dslcd.println(DriverStationLCD.Line.kMain6, 1, "instantiating Bot2011");

        //adds the drive options to the sendable chooser, with TankDrive as the
        //default
        controlChooser = new SendableChooser();
        controlChooser.addDefault("Default (Tank) Drive", new DriveType(TANK_DRIVE));
        controlChooser.addObject("X-Y Drive", new DriveType(TILT_DRIVE));
        controlChooser.addObject("Twist Drive", new DriveType(TWIST_DRIVE));
        SmartDashboard.putData("controlChooser", controlChooser);

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

        driveRobot();
        pickupBalls();
        //shooter code will go here. Make sure to add an implementation for the
        //shooter joystick

        //add automatic code here on the side to line up the robot to shoot
        //put wedge code here.
        updateDashboard();
    }
    /**
     * Updates the dashboard (derp)
     */

    public void driveRobot() {

        if(driveType.getDriveType() == TANK_DRIVE) {
            left = mrJoystick1.getY();
            right = mrJoystick2.getY();
        } else if(driveType.getDriveType() == TILT_DRIVE) {
            left = roundDirectionNum(mrJoystick1.getY() - mrJoystick1.getX()) * (mrJoystick1.getThrottle() + 1) / 2;
            right = roundDirectionNum(-mrJoystick1.getY() - mrJoystick1.getX());
        } else if(driveType.getDriveType() == TWIST_DRIVE) {
            left = driveStick.getX() + driveStick.getTwist();
            right = driveStick.getX() - driveStick.getTwist();
        }

        driveTrain.setSpeed(left, right);
    }

    public void pickupBalls() {

        if(driveType.getDriveType() != TANK_DRIVE) {
            mrJoystick3 = mrJoystick2;
        }

        if(mrJoystick3.getRawButton(5)) {
            ballPickup.forward();
        } else if(mrJoystick3.getRawButton(7) && !mrJoystick3.getRawButton(5)){
            ballPickup.reverse();
        }
    }

    public void feedBalls() {

        if(driveType.getDriveType() != TANK_DRIVE) {
            mrJoystick3 = mrJoystick2;
        }

        if(mrJoystick3.getRawButton(6)) {
            mrLift.up();
        } else if(mrJoystick3.getRawButton(8) && !mrJoystick3.getRawButton(6)){
            mrLift.down();
        }
    }

    public void setWheelSpeed() {

        if(driveType.getDriveType() != TANK_DRIVE) {
            mrJoystick3 = mrJoystick2;
        }

        if(mrJoystick3.getRawButton(1)) {
            mrShooter.setWheelSpeed(0);
        } else if(mrJoystick3.getRawButton(2)) {
            mrShooter.setWheelSpeed(1/3);
        } else if(mrJoystick3.getRawButton(3)) {
            mrShooter.setWheelSpeed(2/3);
        } else if(mrJoystick3.getRawButton(4)) {
            mrShooter.setWheelSpeed(1);
        }
    }

    public void setWheelMode() {

        if(driveType.getDriveType() != TANK_DRIVE) {
            mrJoystick3 = mrJoystick2;
        }

//        if(mrJoystick3.getAxisChannel(Joystick.AxisType.kNumAxis)) {
//
//        }
    }

    public void updateDashboard()
    {
        dslcd.println(DriverStationLCD.Line.kMain6, 1,
        "Output to left side: " + driveStick.getX() + driveStick.getTwist());
        dslcd.println(DriverStationLCD.Line.kUser2, 1,
        "Output to right side: " + (driveStick.getX() - driveStick.getTwist()));
    }

    public class DriveType {
        public int driveType;
        public DriveType(int dType) {
            driveType = dType;
        }

        public int getDriveType() {
            return driveType;
        }
    }

    public double roundDirectionNum(double num) {
        if (num > 1) {
            num = 1;

        }
        if (num < -1) {
            num = -1;

        }
        return num;
    }
}
