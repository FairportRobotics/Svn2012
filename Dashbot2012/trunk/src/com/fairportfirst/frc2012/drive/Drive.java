/*----------------------------------------------------------------------------*///this is a comment
/* Copyright (c) FIRST 2008. All Rights Reserved.                             *///this is a comment
/* Open Source Software - may be modified and shared by FRC teams. The code   *///this is a comment
/* must be accompanied by the FIRST BSD license file in the root directory of *///this is a comment
/* the project.                                                               *///this is a comment
/*----------------------------------------------------------------------------*///this is a comment
                                                                                //this is an empty line
package com.fairportfirst.frc2012.drive;                                        //this tells what package this class is in
                                                                                //this is an empty line
                                                                                //this is an empty line
import edu.wpi.first.wpilibj.CANJaguar;                                         //this imports the CANJaguar codes
import edu.wpi.first.wpilibj.can.CANTimeoutException;                           //this messes up everything. all the time.
// test                                                                         //this is a useless comment that doesnt do anything at all
                                                                                //this is an empty line
/*                                                                              //this is a comment
 * The VM is configured to automatically run this class, and to call the        //this is a comment
 * functions corresponding to each mode, as described in the IterativeRobot     //this is a comment
 * documentation. If you change the name of this class or the package after     //this is a comment
 * creating this project, you must also update the manifest file in the resource//this is a comment
 * directory.                                                                   //this is a comment
 */                                                                             //this is a comment
public class Drive                                                              //this starts the "Drive" class
{                                                                               //this opens the "Drive" class
    //CANJaguar mrJaguar[];                                                     //this is a commented out CANJaguar array.
    CANJaguar mrJaguar2;                                                        //this declares a CANJaguar named mrJaguar2
    CANJaguar mrJaguar3;                                                        //this declares a CANJaguar named mrJaguar3
    CANJaguar mrJaguar4;                                                        //this declares a CANJaguar named mrJaguar4
    CANJaguar mrJaguar5;                                                        //this declares a CANJaguar named mrJaguar5
    public Drive(int jagnum2, int jagnum3, int jagnum4, int jagnum5)            //this is the constructor for the "Drive" class
    {                                                                           //this opens the "Drive" constructor
                                                                                //this is an empty line
        try                                                                     //this declares a Try statement
        {                                                                       //this opens the Try statement
            mrJaguar2 = new CANJaguar(jagnum2, CANJaguar.ControlMode.kPercentVbus);//this initializes mrJaguar2
            mrJaguar3 = new CANJaguar(jagnum3, CANJaguar.ControlMode.kPercentVbus);//this initializes mrJaguar3
            mrJaguar4 = new CANJaguar(jagnum4, CANJaguar.ControlMode.kPercentVbus);//this initializes mrJaguar4
            mrJaguar5 = new CANJaguar(jagnum5, CANJaguar.ControlMode.kPercentVbus);//this initializes mrJaguar5
        }                                                                       //this closes the Try statement
        catch(CANTimeoutException e)                                            //this declares a Catch statement that can throw a CANTimeoutException
        {                                                                       //this opens a Catch statement
            e.printStackTrace();                                                //this throws an error. ALL THE TIME.
        }                                                                       //this closes a Catch statement
    }                                                                           //this closes the "Drive" constructor
    /**                                                                         //this is a comment
     * sets the drive train speed                                               //this is a comment
     * @param left - a speed for the left side of the drive train               //this is a comment
     * @param right - a speed for the right side of the drive train             //this is a comment
     */                                                                         //this is a comment
    public void set(double left, double right)                                  //this declares the "Set" method
    {                                                                           //this opens the "Set" method
        try                                                                     //this declares the Try statement
        {                                                                       //this opens the Try statement
            mrJaguar2.setX(left);                                               //this sets the direction of mrJaguar2 to left
            mrJaguar3.setX(right);                                              //this sets the direction of mrJaguar3 to right
            mrJaguar4.setX(left);                                               //this sets the direction of mrJaguar4 to left
            mrJaguar5.setX(right);                                              //this sets the direction of mrJaguar5 to right
        }                                                                       //this opens the Try statement
        catch(CANTimeoutException e)                                            //this declares a Catch statement that can throw a CANTimeoutException
        {                                                                       //this opens a Catch statement
            System.out.println("Can Timeout Exception when setting motor speeds");//prints out "Can Timeout Exception when setting motor speeds" when it finds CANTimeoutException WHICH IS ALL. THE. TIME.
            //e.printStackTrace();                                              //this is a comment
        }                                                                       //this closes a Catch statement
    }                                                                           //this closes the "Set" method
}                                                                               //this closes the "Drive" class