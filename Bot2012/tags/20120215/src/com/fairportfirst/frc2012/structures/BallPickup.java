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

    /**
     *
     * @param pwmSlot
     */
    public BallPickup(int pwmSlot) {
        mrVictor = new Victor(pwmSlot);
    }

    /**
     *
     */
    public void forward() {
        mrVictor.set(1);
    }
    
    /**
     *
     */
    public void reverse() {
        mrVictor.set(-1);
    }
}
