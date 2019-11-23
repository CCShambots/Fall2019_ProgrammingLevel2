/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DefaultDrivetrainCommand extends Command {

  private Joystick driverJoystick;

  public DefaultDrivetrainCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    driverJoystick = Robot.m_oi.getDriverJoystick();
    requires(Robot.my_drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  // Creating variables to hold the joystick values
  double joystick_left_value = 0;
  double joystick_right_value = 0;

  // Getting the joystick values
  joystick_left_value = driverJoystick.getRawAxis(RobotMap.DRIVER_LEFT_Y_AXIS) * -1 ;
  joystick_right_value = driverJoystick.getRawAxis(RobotMap.DRIVER_RIGHT_Y_AXIS) * -1 ;

  // Setting the subsystem speeds to the joystick values
  Robot.my_drive.setLeftSpeed(joystick_left_value);
  Robot.my_drive.setRightSpeed(joystick_right_value);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.my_drive.stop(); // The robot should stop moving if this command is ended
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.my_drive.stop(); // The robot should stop moving if this command is interrupted
  }
}
