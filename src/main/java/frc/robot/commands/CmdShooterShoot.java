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

public class CmdShooterShoot extends CommandBase {
  /**
   * Creates a new CmdDefaultShooterShoot.
   */
  private final Magazine magazine;
  private final Shooter shooter;

  public CmdShooterShoot(Magazine magazine, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.magazine = magazine;
    this.shooter = shooter;
    
    addRequirements(magazine, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.getShooterHighSpeed())
    {
      shooter.topShooterTalonSRX.set(ControlMode.Velocity, shooter.stsShooterUpperShooterHighSpeed);
      shooter.bottomShooterTalonSRX.set(ControlMode.Velocity, shooter.stsShooterLowerShooterHighSpeed);
    }
    else if(shooter.getShooterLowSpeed())
    {
      shooter.topShooterTalonSRX.set(ControlMode.Velocity, shooter.stsShooterUpperShooterLowSpeed);
      shooter.bottomShooterTalonSRX.set(ControlMode.Velocity, shooter.stsShooterLowerShooterLowSpeed);
    }
    
    //shooter.preigniterSolenoid.set(false);
    System.out.println("Welcome to the shooting command");
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
