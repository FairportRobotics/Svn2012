/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.fairportfirst.frc2012.drive;
// Stefen was here and he's testing to see if he can commit just fine.
// Stefen was here, yet again.
import edu.wpi.first.wpilibj.Dashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.camera.AxisCamera;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Fanbot2012 extends IterativeRobot
{
    //CANJaguar mrJaguar[];
    //CANJaguar mrJaguar4;
    //CANJaguar mrJaguar5;
    Drive mrDriveTrain;
    DriverStationLCD dslcd;
    Joystick mrJoystick;
    Shooter mrShooter;
    AxisCamera camera;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
        dslcd = DriverStationLCD.getInstance();
        dslcd.println(DriverStationLCD.Line.kMain6, 1, "instantiating Bot2011");
        //comment out the line which isn't being used
        mrDriveTrain = new Drive(2,3);//change the port numbers if necessary
        mrShooter = new Shooter(2,3);

        mrJoystick = new Joystick(1);
    }
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic()
    {

    }
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic()
    {
        //comment the method out if you aren't testing it
        driveTest();
        //shooterTest();
    }
    private void shooterTest()
    {


        if (!mrJoystick.getButton(Joystick.ButtonType.kTrigger)){
            double JagXVal;
            JagXVal = -mrJoystick.getY();
            mrShooter.setEachJags(JagXVal, JagXVal);
        }else
            mrShooter.setBothJagsX(roundDirectionNum(mrJoystick.getX()));
    }
    private void driveTest()
    {
        double left = roundDirectionNum(-mrJoystick.getY()+mrJoystick.getX());
        double right =  roundDirectionNum(-mrJoystick.getY()-mrJoystick.getX());
        mrDriveTrain.set(left, right);     
        //camera = AxisCamera.getInstance("10.5.78.11");
        //OK FOR FINAL THING 2ND PRT USE 192.168.0.90 IF USE SWITCH OR BRIGE USE 10.5.78.11
        //camera.writeResolution(AxisCamera.ResolutionT.k320x240);
        //camera.writeBrightness(25);
        dslcd.println(DriverStationLCD.Line.kUser2, 1, "Joystick X: " + mrJoystick.getX());
        dslcd.println(DriverStationLCD.Line.kUser3, 1, "Joystick Y: " + mrJoystick.getY());
        dslcd.println(DriverStationLCD.Line.kUser4, 1, "Left set value: " + left);
        dslcd.println(DriverStationLCD.Line.kUser5, 1, "Right set value: " + right);
        dslcd.updateLCD();
        updateDashBoard();
    }

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
        return num;
    }
}