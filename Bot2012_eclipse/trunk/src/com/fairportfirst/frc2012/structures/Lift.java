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

    /**
     *
     * @param pwmSlot
     */
    public Lift(int pwmSlot) {
        mrVictor = new Victor(pwmSlot);
    }

    /**
     *
     */
    public void up() {
        mrVictor.set(1);
    }

    /**
     * 
     */
    public void down() {
        mrVictor.set(-1);
    }
}
