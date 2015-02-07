/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.fairportfirst.frc2012.structures;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Victor;

/**
 * The class that controls the Wedge that holds down the bridge
 * @author brennan
 */
public class Wedge
{
    private Victor mrVictor;
    private DigitalInput upLimit;
    private DigitalInput downLimit;
    double motorSpeed = 0.8;
    //double speeder = motorSpeedDown;
    boolean starter = false;

    public Wedge(int dioSlot, DigitalInput uLmt, DigitalInput dLmt) {
        mrVictor = new Victor(dioSlot);
        upLimit = uLmt;
        downLimit = dLmt;
    }

    public void down()
    {
    	if (! downLimit.get())
        {

            mrVictor.set(-motorSpeed/2);
    	}
        else
        {
            reset();
        }
    }
    
    public void up()
    {

    	if (! upLimit.get())
        {
            mrVictor.set(motorSpeed);
    	}
        else
        {
            reset();
        }
    }
    public void reset()
    {
        mrVictor.set(0.0);
    }
    public void reader()
    {
        starter = false;
        System.out.println("Upper Limmet: " + upLimit.get() + "\nLowwer Limmet: " + downLimit.get());
    }
}
