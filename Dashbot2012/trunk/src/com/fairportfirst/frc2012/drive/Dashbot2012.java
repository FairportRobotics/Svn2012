/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.fairportfirst.frc2012.drive;

import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.networktables.NetworkTableKeyNotDefined;
import edu.wpi.first.wpilibj.smartdashboard.*;

import java.util.Date;

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

    DriverStationLCD dslcd;
    DriveSystem ds;

    Joystick mrJoystick1;
    Joystick mrJoystick2;

    NetworkTable prefsTable;

//    AutoSpeed autoSpeed;
    public String shootSpeed = "1";
    public String newSpeed = "0";
    public String shootDelay = "2";

    Button mrButton1;


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
        dslcd.println(DriverStationLCD.Line.kMain6, 1, "instantiating Dash2012");
        
        mrJoystick1 = new Joystick(1);
        mrJoystick2 = new Joystick(2);
        
        prefsTable = NetworkTable.getTable("Preferences");
        prefsTable.putString("Speed", shootSpeed);
        prefsTable.putString("Delay", shootDelay);
//        AutoSpeed autoSpeed = new AutoSpeed(shootSpeed);


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
        public double adjustSensitivity(double x) {
            return (x+1)/2;
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
                    return roundDirectionNum(joystick1.getY()-joystick1.getX())*(joystick1.getThrottle()+1)/2;
                case 3://Twist
                    return roundDirectionNum(joystick1.getY() - joystick1.getTwist())*(joystick1.getThrottle()+1)/2;
                case 4://gamepad tank
                    return joystick1.getRawAxis(2)*(joystick1.getThrottle()+1)/2;
                case 5://gamepad split
                    return roundDirectionNum(joystick1.getRawAxis(2) - joystick1.getRawAxis(4))*(joystick1.getThrottle()+1)/2;
                default://tank
                    return joystick1.getY()*(joystick1.getThrottle()+1)/2;
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
    }
    
    /**
     * This function is called periodically during teleop mode.
     */
    public void teleopContinuous()
    {            
        if(mrJoystick1.getRawButton(1)){
            dslcd.println(DriverStationLCD.Line.kUser2, 1, "Button 1");
        }

        newSpeed = prefsTable.getString("Speed", "11");
        System.out.println("newSpeed: " + newSpeed);
        System.out.println("time: " + new Date().getTime());
//        try {
//            newSpeed = prefsTable.getDouble("speed");
//            //        dslcd.println(DriverStationLCD.Line.kUser4, 1, "speed: " + shootSpeed);
//        } catch (NetworkTableKeyNotDefined ex) {
//            ex.printStackTrace();
//        }
//        dslcd.println(DriverStationLCD.Line.kUser4, 1, "speed: " + shootSpeed);



      Timer.delay(1.0);
      updateDashBoard();
    }



//    public class AutoSpeed
//    {
//        public double autoSpeed;
//
//        public AutoSpeed(double spd)
//        {
//            autoSpeed = spd;
//        }
//
//        public double getSpeed()
//        {
//            return autoSpeed;
//        }
//    }


    public void updateDashBoard()
    {
      if (!newSpeed.equals(shootSpeed)) {
      dslcd.println(DriverStationLCD.Line.kUser3, 1, "new speed: " + newSpeed);
      }
      dslcd.println(DriverStationLCD.Line.kUser4, 1, "shootSpeed: " + shootSpeed);
      String delay = prefsTable.getString("Delay", "22");
      dslcd.println(DriverStationLCD.Line.kUser5, 1, "delay: " + delay);
      dslcd.updateLCD();
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
