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
public class Wedge
{
    private Victor mrVictor;

    public Wedge(int pwmSlot) {
        mrVictor = new Victor(pwmSlot);
    }

    public void wedgeDown() {
        mrVictor.set(1);
    }
    
    public void wedgeUp() {
        mrVictor.set(-1);
    }
}
