/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.fairportfirst.frc2012.drive;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 *
 * @author brennan
 */             

public class Shooter
{
    //alec and , put the code here!!
    CANJaguar mrJaguar1;
    CANJaguar mrJaguar2;

    /** Makes a new constructor for the shooter 
     * @param requires a port number for CAN Jaguar one and two
     * @return cold is the void
     */
    public Shooter(int portNumber1, int portNumber2)
    {
        try
        {
            mrJaguar1 = new CANJaguar(portNumber1, CANJaguar.ControlMode.kPercentVbus);
            mrJaguar2 = new CANJaguar(portNumber2, CANJaguar.ControlMode.kPercentVbus);
        }
        catch (CANTimeoutException ex)
        {
            ex.printStackTrace();
        }
    }
    /**
     * Assigns the values of both jaguar to the inputed double
     * @param jagVal assigns the jaguar one to the inputed double
     */
    public void setBothJagsX(double jagVal)
    {
        try
        {
            mrJaguar1.setX(jagVal);
            mrJaguar2.setX(jagVal);
        }
        catch (CANTimeoutException ex)
        {
            ex.printStackTrace();
        }
    }
    public void setEachJags(double jag1X, double jag2X){
        try
        {
        mrJaguar1.setX(jag1X);
        mrJaguar2.setX(jag2X);
        }
        catch (CANTimeoutException ex)
        {
            ex.printStackTrace();
        }
    }
}
