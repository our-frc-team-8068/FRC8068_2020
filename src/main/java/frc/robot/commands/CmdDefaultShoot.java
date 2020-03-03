/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.LogitechGamePad;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;





public class CmdDefaultShoot extends CommandBase {
  /**
   * Creates a new CmdDefaultShooterShoot.
   */
  private final Magazine magazine;
  private final Shooter shooter;
  private final Joystick driverJoystick;

  double defaultShooterTopShooterPercentage = 1.0;
  double defaultShooterBottomShooterPercentage = 1.0;
  double shootLowTriggerPosition = 0.01;
  double shootHighTriggerPosition = 0.8;

  double bottomShooterSpeed;
  double topShooterSpeed;

  public CmdDefaultShoot(Shooter shooter, Magazine magazine, Joystick driverJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.magazine = magazine;
    this.shooter = shooter;
    this.driverJoystick = driverJoystick;
    
    addRequirements(shooter, magazine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driverJoystick.getRawAxis(LogitechGamePad.RIGHT_TRIGGER) >= shootHighTriggerPosition)
    {
      bottomShooterSpeed = shooter.convertRpmToEncoderCounts(shooter.stsShooterLowerShooterHighSpeed);
      topShooterSpeed = shooter.convertRpmToEncoderCounts(shooter.stsShooterUpperShooterHighSpeed);
    }
    else if(driverJoystick.getRawAxis(LogitechGamePad.RIGHT_TRIGGER) >= shootLowTriggerPosition)
    {
      bottomShooterSpeed = shooter.convertRpmToEncoderCounts(shooter.stsShooterLowerShooterLowSpeed);
      topShooterSpeed = shooter.convertRpmToEncoderCounts(shooter.stsShooterUpperShooterLowSpeed);
    }
    else
    {
      bottomShooterSpeed = 0.0;
      topShooterSpeed = 0.0;
    }
    if(bottomShooterSpeed == 0.0 && topShooterSpeed == 0.0)
    {
      shooter.topShooterTalonSRX.set(ControlMode.PercentOutput, 0.0);
      shooter.bottomShooterTalonSRX.set(ControlMode.PercentOutput, 0.0);
     // shooter.retractPreignitor();
      shooter.setPreigniterSpeed(0.0);
    }
    else
    {
      shooter.topShooterTalonSRX.set(ControlMode.Velocity, topShooterSpeed);
      shooter.bottomShooterTalonSRX.set(ControlMode.Velocity, bottomShooterSpeed);
      shooter.setPreigniterSpeed(0.75);
     // shooter.extendPreignitor();
    }

    //shooter.topShooterTalonSRX.set(ControlMode.PercentOutput, defaultShooterTopShooterPercentage);
    //shooter.bottomShooterTalonSRX.set(ControlMode.PercentOutput, defaultShooterBottomShooterPercentage);
    
    //shooter.preigniterSolenoid.set(false);
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
