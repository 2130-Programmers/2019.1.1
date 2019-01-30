package org.usfirst.frc2130.DemoLeveler.subsystems;

import org.usfirst.frc2130.DemoLeveler.commands.*;
import org.usfirst.frc2130.DemoLeveler.subsystems.NavXSubsystem;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

public class Climber extends PIDSubsystem {

    private Talon front;
    private Talon back;
    private NavXSubsystem navX;


    // Initialize your subsystem here
    public Climber(NavXSubsystem navX) {
        super("Climber", 1.0, 0.0, 0.0);

        //Set local refernce to NavX for getting pitch command
        this.navX = navX;

        setAbsoluteTolerance(0.2);
        getPIDController().setContinuous(false);
        getPIDController().setName("Climber", "PIDSubsystem Controller");
        LiveWindow.add(getPIDController());


        front = new Talon(0);
        addChild("Front", front);
        front.setInverted(false);
        
        back = new Talon(1);
        addChild("Back", back);
        back.setInverted(false);
        


        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }

    @Override
    public void initDefaultCommand() {



        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    @Override
    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;

        return navX.pidGet();

    }

    @Override
    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);

        front.pidWrite(output);

    }




}
