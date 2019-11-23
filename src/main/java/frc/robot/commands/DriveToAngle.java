/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveToAngle extends Command {

  private double desiredAngle;
  private PIDController angleController;
  private double power;

  public DriveToAngle(double in_desiredAngle) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.my_drive);

    desiredAngle = in_desiredAngle;
    power = 0;
    SmartDashboard.putNumber("AnglePIDpower", power);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    angleController = Robot.my_drive.getPidController();
    angleController.setSetpoint(desiredAngle);
    angleController.setAbsoluteTolerance(RobotMap.TOLERANCE);
    angleController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    power = angleController.get();

    SmartDashboard.putNumber("AnglePIDpower", power);
    Robot.my_drive.setLeftSpeed(power);
    Robot.my_drive.setRightSpeed(-power);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return angleController.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.my_drive.stop();
    angleController.disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.my_drive.stop();
    angleController.disable();
  }
}
