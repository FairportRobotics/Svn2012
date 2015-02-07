/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.fairportfirst.frc2012.structures;

import edu.wpi.first.wpilibj.Victor;

/**
 *
 * @author Stefen
 */
public class Lift {
    private Victor mrVictor;
    double speed = 1.0;

    /**
     *
     * @param pwmSlot
     */
    public Lift(int dioSlot) {
        mrVictor = new Victor(dioSlot);
    }

    /**
     *
     */
    public void up() {
        mrVictor.set(speed);
    }

    /**
     * 
     */
    public void down() {
        mrVictor.set(-speed);
    }
    public void reset()
    {
        mrVictor.set(0.0);
    }
}
