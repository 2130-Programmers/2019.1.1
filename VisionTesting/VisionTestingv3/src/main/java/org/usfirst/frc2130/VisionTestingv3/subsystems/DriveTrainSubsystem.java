// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc2130.VisionTestingv3.subsystems;


import org.usfirst.frc2130.VisionTestingv3.Robot;
import org.usfirst.frc2130.VisionTestingv3.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class DriveTrainSubsystem extends Subsystem {

    public double errorPrevious = 0;
    public double limelightX;
    public double limelightY;
    public double limelightArea;
    public NetworkTable table;
    public NetworkTableEntry tx;
    public NetworkTableEntry ty;
    public NetworkTableEntry ta;
    public boolean correctAngle = false;
    public double movingOutput;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
    
    private AHRS ahrs;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private WPI_TalonSRX lRMotor;
    private WPI_VictorSPX lFMotor;
    private SpeedControllerGroup leftMotors;
    private WPI_TalonSRX rRMotor;
    private WPI_VictorSPX rFMotor;
    private SpeedControllerGroup rightMotors;
    private DifferentialDrive differentialDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public DriveTrainSubsystem() {

        try {
    		ahrs = new AHRS(SPI.Port.kMXP);
    	} catch (RuntimeException ex ) {
    		DriverStation.reportError("Error instantiating navX-MXP" + ex.getMessage(), true);
        } 

        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        lRMotor = new WPI_TalonSRX(1);
        
        
        
        lFMotor = new WPI_VictorSPX(3);
        
        
        
        leftMotors = new SpeedControllerGroup(lRMotor, lFMotor  );
        addChild("LeftMotors",leftMotors);
        
        
        rRMotor = new WPI_TalonSRX(2);
        
        
        
        rFMotor = new WPI_VictorSPX(4);
        
        
        
        rightMotors = new SpeedControllerGroup(rRMotor, rFMotor  );
        addChild("RightMotors",rightMotors);
        
        
        differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
        addChild("DifferentialDrive",differentialDrive);
        differentialDrive.setSafetyEnabled(true);
        differentialDrive.setExpiration(0.1);
        differentialDrive.setMaxOutput(1.0);

        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    lFMotor.setInverted(true);
    rFMotor.setInverted(true);
    lRMotor.setInverted(true);
    rRMotor.setInverted(true);

    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new LimelightVisionSwitcher());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

        limelightX = tx.getDouble(0.0);
        limelightY = ty.getDouble(0.0);
        limelightArea = ta.getDouble(0.0);  
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    /* Autonomous Commands */
    /*
    public double ahrsAngle() {
        return ahrs.getAngle();
    }

    public void zeroTheSensors() {
        ahrs.reset();
        errorPrevious = 0;
    }

    

    public double controller(double target) {
    	double tolerance = 10; // The robot will try to get within 2 degrees of the desired angle
    	double kP = 0.021; // The proportional gain of our P loop
    	//double kI = 0.0001;
    	double kD = 0.0000197;
    	
    	double dIteration = 0.02;
    	
    	//double iOutput;
    	double dOutput;
    	double error = target;
    	
    	//iGenerator += (error * 0.1);
    	//iOutput = iGenerator * kI;
    	
    	dOutput = (error - errorPrevious) / dIteration;
    	
    	errorPrevious = error;

    	if (Math.abs(error) > tolerance) {
            correctAngle = false;
    		return (error * kP) + dOutput * kD; //+ iOutput;
    	} else {
            correctAngle = true;
    		return 0.0;
    	}
    }

    public boolean correctAngleValue() {
        return correctAngle;
    }

    public void applyTurningPower(double output) {
    	lRMotor.set(output);
    	lFMotor.set(output);
    	rRMotor.set(output);
    	rFMotor.set(output);
    }

    public void applyMovingPower(double output) {
    	lRMotor.set(output);
    	lFMotor.set(output);
    	rRMotor.set(-output);
    	rFMotor.set(-output);
    }

    public void turnTo(double turnAngle) {
    	applyTurningPower(controller(turnAngle));
    }

    public void turnToLimelightTarget() {
        turnTo(limelightX);
    }

    public double moveToUsingArea() {
        
    	double tolerance = 40; // The robot will try to get 35 inches from target
        /*double H1 = 41;
        double H2 = 62;
        double A1 = 10;
        double A2 = current;
        
        double output = (H2 - H1) / Math.abs((Math.tan(A1 + A2)));
        */
        /*
        double kA = 18.2;
        double kB = -71.8;
        double kC = 95.8;
        double X = current;
        double squaredX = Math.pow(X, 2);

        double output = movingOutputValue(kA, kB, kC, X, squaredX);
        */
        /*
        double k = 44;

        double output = k/Math.sqrt(limelightArea);

    	if (Math.abs(output) > tolerance) {
    		return output;
    	} else {
    		return 0.0;
        }
        
    }

    public double movingOutputValue(double A, double B, double C, double X, double x2) {
        movingOutput = (A * x2) + (B * X) + (C);
        return movingOutput;
    }

    public double movingOutputDisplay() {
        return movingOutput;
    }

    public void moveTo(double target) {
        applyMovingPower(moveToUsingArea());
    }

    public void moveToLimelightTarget() {
        moveTo(limelightArea);
    }






    
    /* User Controlled Commands */

    public void setPipeline(int pipeline) {
		NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
    	pipelineEntry.setNumber(pipeline);
    }

    public void driveWithDifferential() {
        differentialDrive.arcadeDrive(Robot.oi.driverJoy.getRawAxis(1) * -1, 
                                      Robot.oi.driverJoy.getRawAxis(4) * .7);
    }


}

