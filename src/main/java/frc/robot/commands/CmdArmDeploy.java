/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LogitechGamePad;
import frc.robot.LogitechJoystick;
import frc.robot.subsystems.Arm;

public class CmdArmDeploy extends CommandBase {
  /**
   * Creates a new CmdArmDeploy.
   */
  private final Arm arm;
  private final Joystick driverJoystick;
  private final Joystick operatorJoystick;

  public CmdArmDeploy(Arm arm, Joystick driverJoystick, Joystick operatorJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.driverJoystick = driverJoystick;
    this.operatorJoystick = operatorJoystick;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setArmSpeed(0.1);
    if(driverJoystick.getRawButton(LogitechGamePad.BUTTON_A) && operatorJoystick.getRawButton(LogitechJoystick.BUTTON_5))
    {
      arm.setArmSpeed(0.2);
    }
    else
    {
      arm.setArmSpeed(0.0);
    }

    if(driverJoystick.getRawButton(LogitechGamePad.START) && operatorJoystick.getRawButton(LogitechJoystick.BUTTON_12))
    {
      arm.setWinchSpeed(0.2);
    }
    else
    {
      arm.setWinchSpeed(0.0);
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
