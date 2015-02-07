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

    public Wedge(int pwmSlot, DigitalInput uLmt, DigitalInput dLmt) {
        mrVictor = new Victor(pwmSlot);
        upLimit = uLmt;
        downLimit = dLmt;
    }

    public void down() {
    	if (! downLimit.get()) {
            mrVictor.set(1);
    	}
    }
    
    public void up() {
    	if (! upLimit.get()) {
            mrVictor.set(-1);
    	}
    }
}
