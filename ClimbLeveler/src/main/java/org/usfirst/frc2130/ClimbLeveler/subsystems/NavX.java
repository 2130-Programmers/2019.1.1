// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package org.usfirst.frc2130.ClimbLeveler.subsystems;

import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc2130.ClimbLeveler.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;



/**
 *
 */
public class NavX extends Subsystem {

    private AHRS ahrs;

    public NavX() {
    
        try {
    		ahrs = new AHRS(SPI.Port.kMXP);
    	} catch (RuntimeException ex ) {
    		DriverStation.reportError("Error instantiating navX-MXP" + ex.getMessage(), true);
        } 
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public double navXRoll() {
        return ahrs.getRoll();
    }

    public void zeroNavX() {
        ahrs.reset();
    }

	@Override
	protected void initDefaultCommand() {
		
	}

}

