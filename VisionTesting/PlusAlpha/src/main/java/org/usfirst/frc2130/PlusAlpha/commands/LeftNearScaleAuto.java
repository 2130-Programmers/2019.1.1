// RobotBuilder 	Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc2130.PlusAlpha.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc2130.PlusAlpha.subsystems.*;

/**
 *
 */
public class LeftNearScaleAuto extends CommandGroup {


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PARAMETERS
    public LeftNearScaleAuto() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PARAMETERS
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=COMMAND_DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=COMMAND_DECLARATIONS
    	
		addSequential(new DisengageBrake(), 0.1);
		
		addParallel(new ClampCube());
		addSequential(new SScaleMoveOne(), 4.1);
		
		addSequential(new TiltClawDown(), 0.95);
		addSequential(new TiltClawUp(), 0.3);
	
		addSequential(new LSScaleTurnOne(), 1.1);

		addSequential(new ElevatorMediumScaleHeight(), 2);
		
		addParallel(new ClampCube());
		addSequential(new FireCube(), 0.5);
		
		//2ND CUBE
		
		addSequential(new ElevatorIntakeHeight(), 1.3);
		
		addSequential(new LSScaleTurnTwo(), 1.75);
		
		addParallel(new TiltClawDown());
		addParallel(new AutoIntake());
		addSequential(new SScaleMoveThree(), 1.5);
		
		addParallel(new ClampCube());
		addSequential(new TiltClawUp(), 0.2);
		
		addSequential(new SScaleMoveFour(), 1.5);
		
		addSequential(new LSScaleTurnThree(), 1.75);
		
		addSequential(new ElevatorMediumScaleHeight(), 2);
		
		addSequential(new SScaleMoveTwo(), 0.5);
		
		addSequential(new FireCube(), 0.2);
 
    } 
}
