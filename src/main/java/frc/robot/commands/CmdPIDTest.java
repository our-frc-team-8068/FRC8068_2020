/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

public class CmdPIDTest extends CommandBase {
  /**
   * Creates a new cmdPIDTest.
   */

private Magazine magazine;

PIDController pid;

  public CmdPIDTest(Magazine magazine) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.magazine = magazine;

    addRequirements(magazine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PIDController(magazine.getProportionalGain(), 
        magazine.getIntegralGain(), magazine.getDerivativeGain()); 
    pid.enableContinuousInput(0.0, 360.0);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pid.setPID(magazine.getProportionalGain(), 
        magazine.getIntegralGain(), magazine.getDerivativeGain());
        
    magazine.setSpeed(pid.calculate(magazine.getPositionInDegrees(),
        magazine.getSetpointInDegrees()));

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
