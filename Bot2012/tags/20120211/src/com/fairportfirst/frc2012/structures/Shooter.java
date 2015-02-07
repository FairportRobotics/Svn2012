/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.fairportfirst.frc2012.structures;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 * The class that controls the shooter
 * @author brennan
 */
public class Shooter
{
    CANJaguar wheels1;
    CANJaguar wheels2;
    CANJaguar turret;

    double wheelSpeed;

    public Shooter(int wheelJag1, int wheeljag2, int turretJag) {
        try {
            wheels1 = new CANJaguar(6);
        } catch (CANTimeoutException ex) {
            System.out.println("wheels1 Jaguar(6) init failed.");
        }

        try {
            wheels2 = new CANJaguar(7);
        } catch (CANTimeoutException ex) {
            System.out.println("wheels2 Jaguar(7) init failed.");
        }

        try {
            turret = new CANJaguar(8);
        } catch (CANTimeoutException ex) {
            System.out.println("turret Jaguar(8) init failed.");
        }
    }

    public void setWheelSpeed(double speed) {
        wheelSpeed = speed;
    }

    public void wheelsOn() {
        try {
            wheels1.setX(wheelSpeed);
            wheels2.setX(wheelSpeed);
        } catch (CANTimeoutException ex) {
            System.out.println("wheels1, 2 Jaguar(6, 7) wheelsOn failed.");
        }
    }

    public void wheelsOff() {
        try {
            wheels1.setX(0);
            wheels2.setX(0);
        } catch (CANTimeoutException ex) {
            System.out.println("wheels1, 2 Jaguar(6, 7) wheelsOff failed.");
        }
    }

    public void turretClockwise(double clockwise) {
        turret.set(1);
    }
}
