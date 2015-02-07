/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.fairportfirst.frc2012.drive;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
/**
 * This is the code from which the robot drives
 * @author brennan
 */
public class DriveTrain
{
    private final int LEFT_JAG_NUM_1;
    private final int LEFT_JAG_NUM_2;
    private final int RIGHT_JAG_NUM_3;
    private final int RIGHT_JAG_NUM_4;
    private CANJaguar leftJag1;
    private CANJaguar leftJag2;
    private CANJaguar rightJag1;
    private CANJaguar rightJag2;
    /**
     * This constructor is private and should be instead accessed through the
     * getInstance method
     */
    public DriveTrain(int jagnum1, int jagnum2, int jagnum3, int jagnum4)
    {
        LEFT_JAG_NUM_1 = jagnum1;
        LEFT_JAG_NUM_2 = jagnum2;
        RIGHT_JAG_NUM_3 = jagnum3;
        RIGHT_JAG_NUM_4 = jagnum4;
        try
        {
            leftJag1 = new CANJaguar(LEFT_JAG_NUM_1, CANJaguar.ControlMode.kPercentVbus);
            leftJag2 = new CANJaguar(LEFT_JAG_NUM_2, CANJaguar.ControlMode.kPercentVbus);
            rightJag1 = new CANJaguar(RIGHT_JAG_NUM_3, CANJaguar.ControlMode.kPercentVbus);
            rightJag2 = new CANJaguar(RIGHT_JAG_NUM_4, CANJaguar.ControlMode.kPercentVbus);
        }
        catch(CANTimeoutException e)
        {
            System.out.println("There was an error when the Jaguars we're being"
                    + " instantiated\n" + e);
        }
    }
    /**
     * Sets the jaguars on the drive train
     * @param left the variable to set on the left
     * @param right and the right
     */
    public void set(double left, double right)
    {
        try
        {
            leftJag1.setX(left);
            leftJag2.setX(left);
            rightJag1.setX(right);
            rightJag2.setX(right);
        }
        catch(CANTimeoutException e)
        {
            System.out.println("There was an error when the Jaguars we're being"
                    + " set:\n" + e);
        }
        
    }
}
