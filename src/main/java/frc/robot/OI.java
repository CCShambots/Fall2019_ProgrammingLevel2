/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.DriveToDistancePID;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);
  private Joystick driverJoystick;
  private Button a_button;
  private Button b_button;


  public OI() {
    driverJoystick = new Joystick(RobotMap.DRIVER_JOYSTICK_PORT);
    a_button = new JoystickButton(driverJoystick, RobotMap.DRIVER_A_BUTTON);
    b_button = new JoystickButton(driverJoystick, RobotMap.DRIVER_B_BUTTON);

    a_button.whenPressed(new DriveToDistance(20000));
    b_button.whenPressed(new DriveToDistancePID(20000));
  }

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  public Joystick getDriverJoystick() {
    return driverJoystick;
  }

  public void log() {
    SmartDashboard.putNumber("Left Y Axis Input", driverJoystick.getRawAxis(RobotMap.DRIVER_LEFT_Y_AXIS));
    SmartDashboard.putNumber("Right Y Axis Input", driverJoystick.getRawAxis(RobotMap.DRIVER_RIGHT_Y_AXIS));
  }
}
