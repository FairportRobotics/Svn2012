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
    private final int LEFT_JAG_NUM_A;
    private final int RIGHT_JAG_NUM_A;
    private final int LEFT_JAG_NUM_B;
    private final int RIGHT_JAG_NUM_B;
    private CANJaguar leftJagA;
    private CANJaguar rightJagA;
    private CANJaguar leftJagB;
    private CANJaguar rightJagB;
    /**
     * This constructor is private and should be instead accessed through the
     * getInstance method
     */
    public DriveTrain(int jagnum1, int jagnum2, int jagnum3, int jagnum4)
    {
        LEFT_JAG_NUM_A = jagnum1;
        RIGHT_JAG_NUM_A = jagnum2;
        LEFT_JAG_NUM_B = jagnum3;
        RIGHT_JAG_NUM_B = jagnum4;
        try
        {
            leftJagA = new CANJaguar(LEFT_JAG_NUM_A, CANJaguar.ControlMode.kPercentVbus);
            leftJagB = new CANJaguar(LEFT_JAG_NUM_B, CANJaguar.ControlMode.kPercentVbus);
            rightJagA = new CANJaguar(RIGHT_JAG_NUM_A, CANJaguar.ControlMode.kPercentVbus);
            rightJagA.configEncoderCodesPerRev(360);
            rightJagB = new CANJaguar(RIGHT_JAG_NUM_B, CANJaguar.ControlMode.kPercentVbus);
        }
        catch(CANTimeoutException e)
        {
            System.out.println("DriveTrain.DriveTrain(): CANTimeoutException when initializing Jaguars.");
        }
        catch(NullPointerException npe)
        {
            System.out.println("DriveTrain.DriveTrain(): NullPointerException when initializing Jaguars.");
        }
    }
    /**
     * Sets the jaguars on the drive train
     * @param left the variable to set on the left
     * @param right and the right
     */
    public void setSpeed(double left, double right)
    {
        try
        {
            leftJagA.setX(left);
            leftJagB.setX(left);
            rightJagA.setX(right);
            rightJagB.setX(right);
            displayRPM();
        }
        catch(CANTimeoutException e)
        {
            System.out.println("DriveTrain.setSpeed(): CANTimeoutException");
        }
        catch(NullPointerException npe) {
            System.out.println("DriveTrain.setSpeed(): NullPointerException");
        }       
    }
    public void displayRPM()
    {
//        try
//        {
////            System.out.println("-------->>>>>>>> Wheel Speed: " + rightJagA.getSpeed());
//        }
//        catch (CANTimeoutException ex)
//        {
//            ex.printStackTrace();
//        }
    }
}