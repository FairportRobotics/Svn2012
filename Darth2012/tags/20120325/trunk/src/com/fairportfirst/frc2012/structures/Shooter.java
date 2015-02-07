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
     public final double PID_CONSTANT = 0.00002;//was -01
     public final double PID_SAMPLE = 0.3;
     public double currentSpeed = 0.0;
     public final double MAX_RPM = 5100.0;
//     Encoder


     //TODO: //            wheels1.getOutputVoltage(); // max volts for given rpm
    public static double TURRET_SPEED = .3;

    double wheelSpeed;

//    public Shooter(int wheelJag1, int wheeljag2, int turretJag, DigitalInput cwLimit, DigitalInput ccwLimit) {
    public Shooter(int wheelJag1, int wheelJag2, int turretJag) {

        try {
            wheels1 = new CANJaguar(wheelJag1); //,CANJaguar.ControlMode.kVoltage // VOLTAGE MODE
            
        } catch (CANTimeoutException ex) {
            System.out.println("Shooter.Shooter(): CANTimeoutException\nCANJaguar 6");
        }

        try {
            wheels2 = new CANJaguar(wheelJag2);
//            wheels2.changeControlMode(CANJaguar.ControlMode.kSpeed);
            wheels2.configEncoderCodesPerRev(360);
            wheels2.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
//            wheels2.enableControl();
            wheels2.setSpeedReference(CANJaguar.SpeedReference.kEncoder);
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
       currentSpeed = speed;
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
            currentSpeed = 0.0;
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
//            ex.printStackTrace();
              System.out.println("Oops I must be sleeping");
        }
        catch(NullPointerException e)
        {
//            e.printStackTrace();
            System.out.println("Oops I lost the turret");
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
//            System.out.println("SrdRef: " + wheels2.getSpeedReference().value + " OtherREF: " + wheels2.getPositionReference().value);
            return wheels2.getSpeed();
        }
        catch(CANTimeoutException e)
        {
            e.printStackTrace();
            return 0.1;
        }
        catch(NullPointerException e)
        {
            return 0.1;
        }
    }

    public double calRPM(double per)
    {
        return MAX_RPM * per;
    }

    public double calPer(double rpm)
    {
        return rpm/MAX_RPM;
    }
}


//
//            wheels2.configPotentiometerTurns(360);
////            wheels2.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
////            wheels2.enableControl();