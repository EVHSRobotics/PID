package org.usfirst.frc.team2854.robot.oi;

import edu.wpi.first.wpilibj.Joystick;


/**
 * Created by Kevin on 10/23/2015.
 */
public class Axis extends Control{
  private final int id;
  private final double deadband;

  public Axis(Joystick aJoystick, AxisType aAxisType, double aDeadband){
    super(aJoystick);
    id = evaluateAxisType(aAxisType);
    deadband = aDeadband;
  }

  public Axis(Joystick aJoystick, AxisType aAxisType){
    this(aJoystick, aAxisType, 0.05);
  }

  public double get(){
    return joystick.getRawAxis(id);
  }

  public double deadbandGet(){
    double val = get();
    return Math.abs(val) > deadband ? val : 0.0;
  }

}
