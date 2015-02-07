/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.fairportfirst.frc2012.structures;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.DigitalInput;
/**
 * The class that controls the shooter
 * @author brennan
 */
public class Shooter
{
    CANJaguar wheels1;
    CANJaguar wheels2;
    CANJaguar turret;

    private DigitalInput clockwiseLimit;
    private DigitalInput counterClockwiseLimit;
    private static double TURRET_SPEED = .5;

    double wheelSpeed;

    public Shooter(int wheelJag1, int wheeljag2, int turretJag, DigitalInput cwLimit, DigitalInput ccwLimit) {
        try {
            wheels1 = new CANJaguar(6);
        } catch (CANTimeoutException ex) {
            System.out.println("Shooter.Shooter(): CANTimeoutException\nCANJaguar 6");
        }

        try {
            wheels2 = new CANJaguar(7);
        } catch (CANTimeoutException ex) {
            System.out.println("Shooter.Shooter(): CANTimeoutException\nCANJaguar 7");
        }

        try {
            turret = new CANJaguar(8);
        } catch (CANTimeoutException ex) {
            System.out.println("Shooter.Shooter(): CANTimeoutException\nCANJaguar 8");
        }
        
        clockwiseLimit = cwLimit;
        counterClockwiseLimit = ccwLimit;
        
    }

    public void setWheelSpeed(double speed) {
        wheelSpeed = speed;
    }

    public void wheelsOn() {
        try {
            wheels1.setX(wheelSpeed);
            wheels2.setX(wheelSpeed);
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


    public void turretClockwise() {
    	if (! clockwiseLimit.get()) {
            try {
                turret.setX(TURRET_SPEED);
            } catch (CANTimeoutException ex) {
                System.out.println("Shooter.turretClockwise(): CANTimeoutException\nCANJaguar 8");
            } catch(NullPointerException npe) {
                System.out.println("Shooter.turretClockwise(): CANTimeoutException\nCANJaguar 8");
            }
    	}
    }

    public void turretCounterClockwise() {
    	if (! counterClockwiseLimit.get()) {
            try {
                turret.setX(-TURRET_SPEED);
            } catch (CANTimeoutException ex) {
                System.out.println("Shooter.turretCounterClockwise(): CANTimeoutException\nCANJaguar 8");
            } catch(NullPointerException npe) {
                System.out.println("Shooter.turretCounterClockwise(): CANTimeoutException\nCANJaguar 8");
            }
    	}
    }

}
