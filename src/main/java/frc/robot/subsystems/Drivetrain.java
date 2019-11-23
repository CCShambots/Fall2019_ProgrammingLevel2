/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX leftMotor;
  private TalonSRX rightMotor;

  public Drivetrain() {
    leftMotor = new TalonSRX(RobotMap.LEFTMOTOR_CANID);
    rightMotor = new TalonSRX(RobotMap.RIGHTMOTOR_CANID);

    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    rightMotor.setInverted(true);

    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    leftMotor.setSelectedSensorPosition(0);
    rightMotor.setSelectedSensorPosition(0);

    leftMotor.config_kP(0, RobotMap.PID_P);
    leftMotor.config_kI(0, RobotMap.PID_I);
    leftMotor.config_kD(0, RobotMap.PID_D);
    rightMotor.config_kP(0, RobotMap.PID_P);
    rightMotor.config_kI(0, RobotMap.PID_I);
    rightMotor.config_kD(0, RobotMap.PID_D);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DefaultDrivetrainCommand());
  }

  // Take in a double, and set the left motor to that percentage of speed
  public void setLeftSpeed(double in_speed){
    leftMotor.set(ControlMode.PercentOutput, in_speed);
  }

  // Take in a double, and set the right motor to that percentage of speed
  public void setRightSpeed(double in_speed){
    rightMotor.set(ControlMode.PercentOutput, in_speed);
  }

  // This takes a desired position for the left motor and uses the Talon PID to get to that position
  public void setLeftPositionPID(double in_position) {
    leftMotor.set(ControlMode.Position, in_position);
  }

    // This takes a desired position for the right motor and uses the Talon PID to get to that position
  public void setRightPositionPID(double in_position) {
    rightMotor.set(ControlMode.Position, in_position);
  }

  // Set both motor controller's speeds to zero
  public void stop(){
    setLeftSpeed(0);
    setRightSpeed(0);
  }

  // This function returns the encoder position of the left motor controller
  public double getLeftEncoderPosition(){
    return leftMotor.getSelectedSensorPosition();
  }

  // This function returns the encoder position of the right motor controller
  public double getRightEncoderPosition(){
    return rightMotor.getSelectedSensorPosition();
  }

  public void log() {
    SmartDashboard.putNumber("Left Motor Output", leftMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Motor Output", rightMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Left Encoder Value", leftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Encoder Value", rightMotor.getSelectedSensorPosition());
  }

}
