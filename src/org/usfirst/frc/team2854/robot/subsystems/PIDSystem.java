package org.usfirst.frc.team2854.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class PIDSystem extends PIDSubsystem {
	private Encoder encoderE; // this is aakash's suggestion. i think it's pretty good.
	private SpeedController motorA;
	private final double REVOLUTION;

    // Initialize your subsystem here
    public PIDSystem(double p, double i, double d, double revolution, SpeedController aMotorA, Encoder aencoder) {
    	super(p, i, d);
    	REVOLUTION = revolution;
    	motorA = aMotorA;
    	encoderE = aencoder;
    	enable();
    	setSetpoint(getPosition());
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }
    
    public void rotate(double rotation){
    	
    	setSetpoint(getSetpoint() + REVOLUTION * rotation);
    }
    
    public int getRawValue(){
    	return encoderE.getRaw();
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    protected double returnPIDInput() {
    	
    	return encoderE.getRaw();
    	
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
    }
    
    protected void usePIDOutput(double output) {
    
    	System.out.println("output "+output);
    	motorA.set(output);
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    }
}
