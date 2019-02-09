package org.usfirst.frc2130.ClimbLeveler.subsystems;

import org.usfirst.frc2130.ClimbLeveler.commands.*;
import org.usfirst.frc2130.ClimbLeveler.subsystems.NavX;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Climber extends PIDSubsystem {

    private WPI_TalonSRX front;
    private WPI_TalonSRX back;
    private NavX navX;

    private double collectiveOutput;

    // Initialize your subsystem here
    public Climber(NavX navX) {
        // @TODO: These gains need to be updated. Start with P gain.
        super("Climber", 0.05, 0.0, 0.0);

        //Set local refernce to NavX for getting pitch command
        this.navX = navX;

        // Initialize collective output to disable climbing
        collectiveOutput = 0.0;

        setAbsoluteTolerance(3);
        getPIDController().setContinuous(false);
        getPIDController().setName("Climber", "PIDSubsystem Controller");
        LiveWindow.add(getPIDController());

        front = new WPI_TalonSRX(1);
        addChild("Front", front);
        front.setInverted(false);
        
        back = new WPI_TalonSRX(3);
        addChild("Back", back);
        back.setInverted(false);
        
        // Make sure the system is off to start with
        disable();
    }

    @Override
    public void initDefaultCommand() {
        // Unused. No default command should be set for this subsystem.
    }

    @Override
    protected double returnPIDInput() {
        // This may need to be inverted to get the correct direction.
        //
        // Assume here that positive pitch means the the front of the robot
        // is tilted up, that means that the front motor needs to slow down.
        return navX.navXRoll();

    }

    @Override
    protected void usePIDOutput(double output) {
        // The PID output represents the controller's attempt to compensate for
        // perceived pitch error. Following our convention for positive pitch
        // meaning the robot is tilted up, for a simple P contoller:
        //
        //      output = Kp * error = Kp * (target - actual)
        //      output = Kp * (0 - pitch)
        //
        // So we see that if there is positive pitch, the controller output will
        // be negative. That means we should ADD the output of the controller to the
        // front motor and SUBTRACT the output from the back motor.

        // Add controller output on front to decrease speed if pitch is positive
        front.set(collectiveOutput + output);

        // Subtract controller output on back to increase speed if pitch is positive
        back.set(collectiveOutput - output);
        
    }

    public void startClimb(double collectiveCommand) {
        // Starts climbing by setting the collective command and enabling the controller.
        //
        // By default, we target 0 tilt. If we want to get fancy we may want to maintain the
        // orientation the robot is when it starts to climb by remembering the pitch when the
        // command is sent. It may be more desirable for it to self-level in the end though.

        // Target 0 pitch
        setSetpoint(0.0);

        // Set the internal collective output
        collectiveOutput = collectiveCommand;

        // Start the PID controller
        enable();
    }

    public void stopClimb() {
        // Turn everything off
        disable();
        front.set(0.0);
        back.set(0.0);
        collectiveOutput = 0.0;
    }

}
