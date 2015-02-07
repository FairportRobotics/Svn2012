/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.fairportfirst.frc2012.drive;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 * @author 578
 */
public class Devices2012 extends IterativeRobot
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
    Joystick mrJoystick1;

    Button mrButton1;
    Relay mrRelay;
    SendableGyro mrGyro;

    SendableChooser controlChooser;
    Shooter mrShooter;
    SmartDashboard mrSmart;


    DriveType driveType;

	// Local variables to count the number of periodic loops performed
	int m_autoPeriodicLoops;
	int m_disabledPeriodicLoops;
	int m_telePeriodicLoops;

        Counter mrCounter;
        
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
        NetworkTable.initialize();
        //mrSmart = new SmartDashboard();
        dslcd = DriverStationLCD.getInstance();
        dslcd.println(DriverStationLCD.Line.kMain6, 1, "instantiating Devices2012");
        
        // If you are not using the method for testing purposes, please comment them out.
        mrJoystick1 = new Joystick(1);
        mrRelay = new Relay(1,1);
        mrGyro = new SendableGyro(1);
        mrGyro.setSensitivity(.007);

        mrCounter = new Counter(1, 1);
        mrCounter.reset();


        int encoderInfo = mrCounter.get();
        SmartDashboard.putInt("Encoder", encoderInfo);

        //adds the drive options to the sendable chooser, with TankDrive as the
        //default
        controlChooser = new SendableChooser();
        controlChooser.addDefault("1 - Toggle on/off", new Cathode(1));
        controlChooser.addObject("2 - Strobe", new Cathode(2));

        SmartDashboard.putData("controlChooser", controlChooser);
        SmartDashboard.putDouble("Gyro Angle", mrGyro.getAngle());
    }
    /**
     * This class manages the joysticks and determines an output for the left
     * and the right regardless of the type of command selected.
     */
    public class Cathode
    {
      int cathodeMode = 1;
        public Cathode(int mode)
        {
            cathodeMode = mode;
        }
    }
    /**
     * This function is called periodically during autonomous mode.
     */
    public void autonomousPeriodic()
    {
        m_autoPeriodicLoops = 0;				// Reset the loop counter for autonomous mode

    }
    
    /**
     * This function is called periodically during teleop mode.
     */
    public void teleopPeriodic()
    {
        if(mrJoystick1.getRawButton(1)){
            mrRelay.set(Relay.Value.kForward);
            dslcd.println(DriverStationLCD.Line.kMain6, 1, "Joystick Pressed.     ");
            dslcd.updateLCD();
        }
        else {
            mrRelay.set(Relay.Value.kReverse);
            dslcd.println(DriverStationLCD.Line.kMain6, 1, "Joystick NOT Pressed.");
            dslcd.updateLCD();
        }



        SmartDashboard.putDouble("Gyro", roundTwoPlaces(mrGyro.getAngle()));
        updateDashboard();


        int encoderInfo = mrCounter.get();
        SmartDashboard.putInt("Encoder", encoderInfo);
    }
    
    public void updateDashboard() {
        dslcd.updateLCD();
    }




    public void disabledInit() {
	m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
//	ClearSolenoidLEDsKITT();
    }
    

    /**
     * 
     */
    public void updateDashBoard()
    {

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


    public double roundTwoPlaces(double dbl) {
        return (double) (int) ((dbl + 0.005) * 100.0) / 100.0;
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
}
