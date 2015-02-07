/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.fairportfirst.frc2012.structures;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
/**
 * The class that controls the shooter
 * @author brennan
 */
public class Shooter
{
    public CANJaguar wheels1;
    public CANJaguar wheels2;
     public CANJaguar turret;

    public static double TURRET_SPEED = .25;

    double wheelSpeed;

//    public Shooter(int wheelJag1, int wheeljag2, int turretJag, DigitalInput cwLimit, DigitalInput ccwLimit) {
    public Shooter(int wheelJag1, int wheelJag2, int turretJag) {

        try {
            wheels1 = new CANJaguar(wheelJag1);
        } catch (CANTimeoutException ex) {
            System.out.println("Shooter.Shooter(): CANTimeoutException\nCANJaguar 6");
        }

        try {
            wheels2 = new CANJaguar(wheelJag2);
        } catch (CANTimeoutException ex) {
            System.out.println("Shooter.Shooter(): CANTimeoutException\nCANJaguar 7");
        }

        try {
            turret = new CANJaguar(turretJag);
        } catch (CANTimeoutException ex) {
            System.out.println("Shooter.Shooter(): CANTimeoutException\nCANJaguar 8");
        }
    }

    public void setWheelSpeed(double speed) {
        try {
            wheels1.setX(speed);
            wheels2.setX(speed);
        } catch (CANTimeoutException ex) {
            System.out.println("Shooter.wheelsOn(): CANTimeoutException\nCANJaguar 6 and CANJaguar 7");
        } catch(NullPointerException npe) {
            System.out.println("Shooter.wheelsOn(): CANTimeoutException\nCANJaguar 6 and CANJaguar 7");
        }
    }

    public void wheelsOff() {
        try {
            wheels1.setX(0);
            wheels2.setX(0);
        } catch (CANTimeoutException ex) {
            System.out.println("Shooter.wheelsOff(): CANTimeoutException\nCANJaguar 6 and CANJaguar 7");
        } catch(NullPointerException npe) {
            System.out.println("Shooter.wheelsOff(): CANTimeoutException\nCANJaguar 6 and CANJaguar 7");
        }
    }


    public void turretClockwise(double speed) {

      boolean clockwiseOK = true;
      
      try {
      clockwiseOK = turret.getForwardLimitOK();
      }
      catch (CANTimeoutException ex) {
                System.out.println("Shooter.getForwardLimitOK: CANTimeoutException\nCANJaguar 8");
      }
    	if (clockwiseOK) {
            try {
                turret.setX(speed);
            } catch (CANTimeoutException ex) {
                System.out.println("Shooter.turretClockwise(): CANTimeoutException\nCANJaguar 8");
            } catch(NullPointerException npe) {
                System.out.println("Shooter.turretClockwise(): CANTimeoutException\nCANJaguar 8");
            }
    	}
    }
    public void turretStop()
    {
        try 
        {
            turret.setX(0.0);
        } 
        catch (CANTimeoutException ex) 
        {
            ex.printStackTrace();
        }
    }


    public void turretCounterClockwise(double speed) {
      boolean counterClockwiseOK = true;

      try {
      counterClockwiseOK = turret.getReverseLimitOK();
      }
      catch (CANTimeoutException ex) {
                System.out.println("Shooter.getReverseLimitOK: CANTimeoutException\nCANJaguar 8");
      } catch(NullPointerException npe) {
                System.out.println("Shooter.getReverseLimitOK: NullPointerException\nCANJaguar 8");
      }
    	if (counterClockwiseOK) {
            try {
                turret.setX(-speed);
            } catch (CANTimeoutException ex) {
                System.out.println("Shooter.turretCounterClockwise(): CANTimeoutException\nCANJaguar 8");
            } catch(NullPointerException npe) {
                System.out.println("Shooter.turretCounterClockwise(): CANTimeoutException\nCANJaguar 8");
            }
    	}
    }
    /**
     * Gets the wheel speed in RPM
     * @return the wheel speed from the encoder on the Jaguar for the shooter
     */
    public double getWheelSpeed()
    {
        try
        {
            return wheels1.getSpeed();
        }
        catch(CANTimeoutException e)
        {
            System.out.println("There was a CanTimeout exception when the "
                    + "encoder was being accessed");
            return 0;
        }
    }
}