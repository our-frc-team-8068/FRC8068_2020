/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LogitechGamePad;
import frc.robot.Utilities;
import frc.robot.subsystems.DriveTrain;

/**
 * An example command that uses an example subsystem.
 */
public class CmdDefaultJoystickDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveTrain driveTrain;
  private final Joystick driverJoystick;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CmdDefaultJoystickDrive(Joystick driverJoystick, DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    this.driverJoystick = driverJoystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // driveTrain.setDriveTrainSpeeds(driverJoystick.getRawAxis(LogitechGamePad.LEFT_Y_AXIS),
    // driverJoystick.getRawAxis(LogitechGamePad.LEFT_Y_AXIS));
    double deadband = 0.1;
    double leftYAxisMagnitude = Utilities.analogScaling(deadband, 1.0, 0.0, 1.0, true,
        Math.abs(driverJoystick.getRawAxis(LogitechGamePad.LEFT_Y_AXIS)));
    double rightXAxisMagnitude = Utilities.analogScaling(deadband, 1.0, 0.0, 1.0, true,
        Math.abs(driverJoystick.getRawAxis(LogitechGamePad.RIGHT_X_AXIS)));

    double joystickAngle =  Math.toDegrees(Math.atan(leftYAxisMagnitude / rightXAxisMagnitude));


        System.out.println("Left Y " + leftYAxisMagnitude + "Right X " + rightXAxisMagnitude);
    if (leftYAxisMagnitude > 0 && rightXAxisMagnitude > 0) 
    {
      // Checks if both the Left Y Axis and Right X Axis are greater than 0.
      // If true moves on to check which quadrent the magnitude value is in.
      if (driverJoystick.getRawAxis(LogitechGamePad.LEFT_Y_AXIS) < 0) 
      {
        // If Left Y Axis is less than 0 the code proceds to check if value is in
        // quadrent I or II.
        if (driverJoystick.getRawAxis(LogitechGamePad.RIGHT_X_AXIS) > 0) 
        {
          // If code proceds here than the value is in quadrent I
          // Will want a positive Y,X Magnitude value.
          System.out.println("You have entered Quadrent I");
          driveTrain.setDriveTrainSpeeds(-leftYAxisMagnitude, (-leftYAxisMagnitude * Math.toDegrees(Math.sin(joystickAngle))));
        } 
        else 
        {
          // If code proceds here than the value is in quadret II
          // Will want a positive X,Y Magnitude value.
          System.out.println("You have entered Quadrent II");
          driveTrain.setDriveTrainSpeeds((-leftYAxisMagnitude * Math.toDegrees(Math.sin(joystickAngle))), -leftYAxisMagnitude);

        }

      } else if (driverJoystick.getRawAxis(LogitechGamePad.LEFT_Y_AXIS) > 0) {
        // If Left Y Axis is greater than 0 the code proceds to check if the value is in
        // quadrent III or IV.
        if (driverJoystick.getRawAxis(LogitechGamePad.RIGHT_X_AXIS) > 0) {
          // If code proceds here than the value is in quadrent IV.
          // Will want a negitive Y,X Magnitude value
          System.out.println("You have entered Quadrent IV");
          driveTrain.setDriveTrainSpeeds(-leftYAxisMagnitude + rightXAxisMagnitude, -rightXAxisMagnitude);
        } else {
          // If code proceds here than the value is in quadrent III.
          // Will want a negitive X,Y Magnitude value.
          System.out.println("You have entered Quadrent III");
          driveTrain.setDriveTrainSpeeds(-rightXAxisMagnitude, -leftYAxisMagnitude + rightXAxisMagnitude);
        }
      }
    } else if (leftYAxisMagnitude == 0 && rightXAxisMagnitude != 0) {
      // Checks if there is no input on the Left Joysticks Y Axis.
      // If true move on to check if we are giving input on the Right joysticks x axis
      if (driverJoystick.getRawAxis(LogitechGamePad.RIGHT_X_AXIS) < 0) {
        // We now know that we want to turn the robot left.
        // Probably want to grab the scaled input and apply it to the proper motors to
        // turn the robot left.
        System.out.println("Robot Turning Left");
        driveTrain.setDriveTrainSpeeds(-rightXAxisMagnitude, rightXAxisMagnitude);
      } else if (driverJoystick.getRawAxis(LogitechGamePad.RIGHT_X_AXIS) > 0) {
        // We know that we want to turn the robot right.
        // Want to grab the scaled input to appy motor power to go right.
        System.out.println("Robot Turning Right");
        driveTrain.setDriveTrainSpeeds(rightXAxisMagnitude, -rightXAxisMagnitude);
      }
    } else if (rightXAxisMagnitude == 0 && leftYAxisMagnitude != 0) {
      // Checks if there is no input on Right Joystick x axis
      // If true moves on to check input values for Left Joystick y axis
      if (driverJoystick.getRawAxis(LogitechGamePad.LEFT_Y_AXIS) < 0) {
        // If raw input for Left Y Axis is greater than 0 than moves on to execute code
        // here.
        // Want to grab scaled vaules to set motors direction forward.
        System.out.println("Robot Moving Forward");
        driveTrain.setDriveTrainSpeeds(leftYAxisMagnitude, leftYAxisMagnitude);
      } else if (driverJoystick.getRawAxis(LogitechGamePad.LEFT_Y_AXIS) > 0) {
        // We know robot needs to drive backwards.
        // Want to grab scaled values to set motors direction backward.
        System.out.println("Robot Moving Backward");
        driveTrain.setDriveTrainSpeeds(-leftYAxisMagnitude, -leftYAxisMagnitude);
      }
    } else {
      // Value is set to 0,0.
      System.out.println("You are at 0,0");
      driveTrain.setDriveTrainSpeeds(0.0, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
