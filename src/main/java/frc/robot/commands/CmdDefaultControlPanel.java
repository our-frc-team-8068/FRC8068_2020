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
import frc.robot.LogitechJoystick;
import frc.robot.Utilities;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.DriveTrain;

/**
 * An example command that uses an example subsystem.
 */
public class CmdDefaultControlPanel extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Joystick operatorJoystick;
  private final ControlPanel controlPanel;

  private boolean firstScan = true;
  private double delayTime = 0.0;
  private double offsetTime = 0.5;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CmdDefaultControlPanel(Joystick operatorJoystick, ControlPanel controlPanel) {
    this.operatorJoystick = operatorJoystick;
    this.controlPanel = controlPanel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(controlPanel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    firstScan = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // driveTrain.setDriveTrainSpeeds(operatorJoystick.getRawAxis(LogitechGamePad.LEFT_Y_AXIS),
    // operatorJoystick.getRawAxis(LogitechGamePad.LEFT_Y_AXIS));
    double deadband = 0.1;
    double twistAxisMagnitude = Utilities.analogScaling(deadband, 1.0, 0.0, 1.0, true,
        Math.abs(operatorJoystick.getRawAxis(LogitechJoystick.TWIST_AXIS)));

    if (twistAxisMagnitude > 0) 
    {
      // Checks if both the Left Y Axis and Right X Axis are greater than 0.
      // If true moves on to check which quadrent the magnitude value is in.
      if (operatorJoystick.getRawAxis(LogitechJoystick.TWIST_AXIS) > 0) 
      {
        controlPanel.setControlPanelSpeed(twistAxisMagnitude);
        firstScan = true;
      }
      else
      {
        if(firstScan)
        {
          delayTime = Timer.getFPGATimestamp() + offsetTime;
          firstScan = false;
        }

        if(Timer.getFPGATimestamp() < delayTime)
        {
          controlPanel.setControlPanelSpeed(-twistAxisMagnitude);
        }
      }
    }
    else
    {
      if(firstScan)
      {
        delayTime = Timer.getFPGATimestamp() + offsetTime;
        firstScan = false;
      }

      if(Timer.getFPGATimestamp() < delayTime)
      {
        controlPanel.setControlPanelSpeed(-0.5);
      }
      else
      {
        controlPanel.setControlPanelSpeed(0.0);
      }
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
