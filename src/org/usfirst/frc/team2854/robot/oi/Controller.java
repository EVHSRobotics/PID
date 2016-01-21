package org.usfirst.frc.team2854.robot.oi;

import edu.wpi.first.wpilibj.Joystick;


/**
 * Created by Kevin on 10/23/2015.
 */
public class Controller{

  public final Joystick joystick;

  public final Button ba;
  public final Button bb;
  public final Button bx;
  public final Button by;
  public final Button blb;
  public final Button brb;
  public final Button bback;
  public final Button bstart;
  public final Button bls;
  public final Button brs;

  public final Axis alx;
  public final Axis aly;
  public final Axis arx;
  public final Axis ary;
  public final Axis alt;
  public final Axis art;

  public Controller(Joystick aJoystick, double aDeadband){
    joystick = aJoystick;

    ba = new Button(joystick, ButtonType.a);
    bb = new Button(joystick, ButtonType.b);
    bx = new Button(joystick, ButtonType.x);
    by = new Button(joystick, ButtonType.y);
    blb = new Button(joystick, ButtonType.lb);
    brb = new Button(joystick, ButtonType.rb);
    bback = new Button(joystick, ButtonType.back);
    bstart = new Button(joystick, ButtonType.start);
    bls = new Button(joystick, ButtonType.ls);
    brs = new Button(joystick, ButtonType.rs);

    alx = new Axis(joystick, AxisType.lx, aDeadband);
    aly = new Axis(joystick, AxisType.ly, aDeadband);
    arx = new Axis(joystick, AxisType.rx, aDeadband);
    ary = new Axis(joystick, AxisType.ry, aDeadband);
    alt = new Axis(joystick, AxisType.lt, aDeadband);
    art = new Axis(joystick, AxisType.rt, aDeadband);
  }

  public Controller(Joystick aJoystick){
    this(aJoystick, 0.05);
  }
}
