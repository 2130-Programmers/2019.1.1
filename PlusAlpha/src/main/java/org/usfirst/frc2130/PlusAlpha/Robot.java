// RobotBuilder Version: 2.0 
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc2130.PlusAlpha;

import edu.wpi.cscore.UsbCamera;   
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot; 
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc2130.PlusAlpha.Robot;
import org.usfirst.frc2130.PlusAlpha.RobotMap;
import org.usfirst.frc2130.PlusAlpha.commands.*;
import org.usfirst.frc2130.PlusAlpha.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {
	
	private static final int IMG_WIDTH = 320;
    private static final int IMG_HEIGHT = 240;


    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

    public static OI oi;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static DrivetrainSubsystem drivetrainSubsystem;
    public static IntakeSubsystem intakeSubsystem;
    public static ElevatorPIDSubsytem elevatorPIDSubsytem;
    public static ClawSubsytem clawSubsytem;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
    	
     	UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    	camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
 
    	
        RobotMap.init();
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        drivetrainSubsystem = new DrivetrainSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        elevatorPIDSubsytem = new ElevatorPIDSubsytem();
        clawSubsytem = new ClawSubsytem();

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();

        /*
        // Add commands to Autonomous Sendable Chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

        chooser.addObject("NoAuto", new NoAuto());
        chooser.addObject("CAuto", new CAuto());
        chooser.addObject("RSideAuto", new RSideAuto());
        chooser.addObject("LSideAuto", new LSideAuto());
        chooser.addObject("WaitForCenterAutoData", new WaitForCenterAutoData());
        chooser.addObject("WaitForRightAutoData", new WaitForRightAutoData());
        chooser.addObject("WaitForLeftAutoData", new WaitForLeftAutoData());
        chooser.addObject("CheckForData", new CheckForData());
        chooser.addObject("CheckForDataLeft", new CheckForDataLeft());
        chooser.addObject("CheckForDataRight", new CheckForDataRight());
        chooser.addObject("SwitchCommand", new SwitchCommand());
        chooser.addObject("LeftScaleCommand", new LeftScaleCommand());
        chooser.addObject("RightScaleCommand", new RightScaleCommand());
        chooser.addObject("LeftSwitchCommand", new LeftSwitchCommand());
        chooser.addObject("RightSwitchCommand", new RightSwitchCommand());
        chooser.addObject("DelayedAutoLineMch12", new DelayedAutoLineMch12());
        chooser.addDefault("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
    	 */
        
        chooser.addObject("SwitchCommand", new SwitchCommand());
        chooser.addObject("LeftScaleCommand", new LeftScaleCommand());
        chooser.addObject("RightScaleCommand", new RightScaleCommand());
        chooser.addObject("LeftSwitchCommand", new LeftSwitchCommand());
        chooser.addObject("RightSwitchCommand", new RightSwitchCommand());
        chooser.addDefault("Autonomous Command", new AutonomousCommand());
        chooser.addObject("DelayedAutoLineMch12", new DelayedAutoLineMch12());



        SmartDashboard.putData("Auto mode", chooser);
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    @Override
    public void disabledInit(){

    }

    @Override
    public void disabledPeriodic() {
    	
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
    	//Robot.clawSubsytem.dropClaw();
    	
        autonomousCommand = chooser.getSelected();
        // schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
      	
    	Robot.elevatorPIDSubsytem.homeEncoder();
    	
    	SmartDashboard.putNumber("Feet Traveled", Robot.drivetrainSubsystem.distanceInFeet());
    	SmartDashboard.putNumber("Robot Speed", Robot.drivetrainSubsystem.drivetrainRPM());
    	SmartDashboard.putNumber("Field Position", Robot.drivetrainSubsystem.getDataValueFromPosition(0));
    	SmartDashboard.putNumber("Change in Angle", Robot.drivetrainSubsystem.ahrsAngle());
    	SmartDashboard.putNumber("Drive Motor Val", Robot.drivetrainSubsystem.motorOutputs());
    	
    	SmartDashboard.putNumber("Elevator Height", Robot.elevatorPIDSubsytem.elevatorHeight());
    	SmartDashboard.putNumber("Resultant Value", Robot.drivetrainSubsystem.resultant);
    	
    	SmartDashboard.putBoolean("Top Prox Value", Robot.elevatorPIDSubsytem.getProx(true));
    	SmartDashboard.putBoolean("Bottom Prox Value", Robot.elevatorPIDSubsytem.getProx(false));
    	
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
    	
    	Robot.clawSubsytem.dropClaw();
    	Robot.intakeSubsystem.zeroX();
    	Robot.drivetrainSubsystem.zeroTheSensors();
    	
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
    	
    	Robot.elevatorPIDSubsytem.homeEncoder();
    	
    	SmartDashboard.putNumber("Feet Traveled", Robot.drivetrainSubsystem.distanceInFeet());
    	SmartDashboard.putNumber("Robot Speed", Robot.drivetrainSubsystem.drivetrainRPM());
    	SmartDashboard.putNumber("Field Position", Robot.drivetrainSubsystem.getDataValueFromPosition(0));
    	SmartDashboard.putNumber("Change in Angle", Robot.drivetrainSubsystem.ahrsAngle());
    	SmartDashboard.putNumber("Drive Motor Val", Robot.drivetrainSubsystem.motorOutputs());
    	
    	SmartDashboard.putNumber("Elevator Height", Robot.elevatorPIDSubsytem.elevatorHeight());
    	SmartDashboard.putNumber("Resultant Value", Robot.drivetrainSubsystem.resultant);
    	
    	SmartDashboard.putBoolean("Top Prox Value", Robot.elevatorPIDSubsytem.getProx(true));
    	SmartDashboard.putBoolean("Bottom Prox Value", Robot.elevatorPIDSubsytem.getProx(false));
    	
        Scheduler.getInstance().run();
    }
}
