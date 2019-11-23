/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;


public class DriveToDistancePID extends Command {

  private double initialLeftPosition;
  private double initialRightPosition;

  private double desiredDistance;
  private double finalLeftPosition;
  private double finalRightPosition;


  public DriveToDistancePID(double in_desiredDistance) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    desiredDistance = in_desiredDistance;
    requires(Robot.my_drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    initialLeftPosition = Robot.my_drive.getLeftEncoderPosition();
    initialRightPosition = Robot.my_drive.getRightEncoderPosition();
    finalLeftPosition = initialLeftPosition + desiredDistance;
    finalRightPosition = initialRightPosition + desiredDistance;

    Robot.my_drive.setLeftPositionPID(finalLeftPosition);
    Robot.my_drive.setRightPositionPID(finalRightPosition);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if ( Robot.my_drive.getLeftEncoderPosition() >= (finalLeftPosition* (1-RobotMap.TOLERANCE)) &&
    Robot.my_drive.getLeftEncoderPosition() <= (finalLeftPosition* (1+RobotMap.TOLERANCE)) &&
    Robot.my_drive.getRightEncoderPosition() >= (finalRightPosition* (1-RobotMap.TOLERANCE)) &&
    Robot.my_drive.getRightEncoderPosition() <= (finalRightPosition* (1+RobotMap.TOLERANCE)) ){
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.my_drive.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.my_drive.stop();
  }
}
