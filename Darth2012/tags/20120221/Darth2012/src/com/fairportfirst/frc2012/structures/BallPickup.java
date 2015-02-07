/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.fairportfirst.frc2012.structures;

import edu.wpi.first.wpilibj.Victor;

/**
 * The class that controls the Wedge that holds down the bridge
 * @author brennan
 */
public class BallPickup
{
    private Victor mrVictor;
    public double pickUpSpeed = 0.5;

    /**
     *
     * @param pwmSlot
     */
    public BallPickup(int pwmSlot)
    {
        mrVictor = new Victor(pwmSlot);
        //mrVictor = new Victor(dioSlot);
    }

    /**
     *
     */
    public void setSpeed(double speed)
    {
        mrVictor.set(speed);
    }
//    public void forward() {
//        mrVictor.set(1.0);
//    }
//
//    /**
//     *
//     */
//    public void reverse()
//    {
//        mrVictor.set(-1.0);
//    }
//    public void reset()
//    {
//        mrVictor.set(0.0);
//    }
}
