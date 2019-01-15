// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc2130.coryDrivingTrains.subsystems;


import org.usfirst.frc2130.coryDrivingTrains.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import org.usfirst.frc2130.coryDrivingTrains.Robot;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class diffDriveSub extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private WPI_TalonSRX lF;
    private WPI_TalonSRX lR;
    private SpeedControllerGroup speedContGroupL;
    private WPI_TalonSRX rF;
    private WPI_TalonSRX rR;
    private SpeedControllerGroup speedContGroupR;
    private DifferentialDrive diffDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public diffDriveSub() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        lF = new WPI_TalonSRX(1);
        
        
        
        lR = new WPI_TalonSRX(2);
        
        
        
        speedContGroupL = new SpeedControllerGroup(lF, lR  );
        addChild("speedContGroupL",speedContGroupL);
        
        
        rF = new WPI_TalonSRX(3);
        
        
        
        rR = new WPI_TalonSRX(4);
        
        
        
        speedContGroupR = new SpeedControllerGroup(rF, rR  );
        addChild("speedContGroupR",speedContGroupR);
        
        
        diffDrive = new DifferentialDrive(speedContGroupL, speedContGroupR);
        addChild("diffDrive",diffDrive);
        diffDrive.setSafetyEnabled(true);
        diffDrive.setExpiration(0.1);
        diffDrive.setMaxOutput(1.0);

        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new driveTrainCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void joystickDriver()
    {

        diffDrive.arcadeDrive(Robot.oi.joyfull.getRawAxis(1), 
                              Robot.oi.joyfull.getRawAxis(0));
    }

    public void enableMotorsFor()
    {
        lF.set(.5);
    }

    public void enableMotorsBack()
    {
        lF.set(-.5);
    }

    public void stopMotors(){
        lF.set(0);
    }

    public void dialDriver()
    {
    }

}

