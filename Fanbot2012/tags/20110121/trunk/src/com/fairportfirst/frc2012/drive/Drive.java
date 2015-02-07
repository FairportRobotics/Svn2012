/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.fairportfirst.frc2012.drive;


import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
// test

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Drive
{
    //CANJaguar mrJaguar[];
    CANJaguar mrLeftJaguar;
    CANJaguar mrRightJaguar;
    //CANJaguar mrJaguar4;
    //CANJaguar mrJaguar5;
    public Drive(int jagnum1, int jagnum2)
    {

        try
        {
            mrLeftJaguar = new CANJaguar(jagnum1, CANJaguar.ControlMode.kPercentVbus);
            mrRightJaguar = new CANJaguar(jagnum2, CANJaguar.ControlMode.kPercentVbus);
        }
        catch(CANTimeoutException e)
        {
            System.out.println(e);
        }
    }
    /**
     * sets the drive train speed
     * @param left - a speed for the left side of the drive train
     * @param right - a speed for the right side of the drive train
     */
    public void set(double left, double right)
    {
        try
        {
            mrLeftJaguar.setX(left);
            mrRightJaguar.setX(right);
        }
        catch(CANTimeoutException e)
        {
            System.out.println(e);
        }
    }
}
