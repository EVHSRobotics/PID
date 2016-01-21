package org.usfirst.frc.team2854.robot.oi;

import edu.wpi.first.wpilibj.Joystick;


/**
 * Created by Kevin on 10/23/2015.
 */
public class Button extends Control{
  private final int id;

  public Button(Joystick aJoystick, ButtonType aButtonType){
    super(aJoystick);
    id = evaluateButtonType(aButtonType);
  }

  public boolean get(){
    return joystick.getRawButton(id);
  }

}
