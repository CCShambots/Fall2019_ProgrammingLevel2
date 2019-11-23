/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  public static final double TOLERANCE = .02;

  public static final int LEFTMOTOR_CANID = 2;
  public static final int RIGHTMOTOR_CANID = 1;

  public static final int DRIVER_JOYSTICK_PORT = 0;

  public static final int DRIVER_LEFT_Y_AXIS = 1;
  public static final int DRIVER_RIGHT_Y_AXIS = 5;

  public static final int DRIVER_A_BUTTON = 1;
  public static final int DRIVER_B_BUTTON = 2;
  public static final int DRIVER_X_BUTTON = 3;

  public static final double PID_P = .5;
  public static final double PID_I = 0;
  public static final double PID_D = 5;

  public static final double gyro_P = .01;
  public static final double gyro_I = .001;
  public static final double gyro_D = 0;
}
