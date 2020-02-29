/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Magazine;

public class CmdCollectorCollect extends CommandBase {
  /**
   * Creates a new CmdCollectorCollect.
   */
  private final Magazine magazine;
  private final Collector collector;
  private double startTime;

  public CmdCollectorCollect(Magazine magazine, Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.magazine = magazine;
    this.collector = collector;
    
    addRequirements(magazine, collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp() + 0.5;
    System.out.println("Starttime: " + startTime);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Current time: " + Timer.getFPGATimestamp());
    if(Timer.getFPGATimestamp() > startTime)
    {
      collector.collect(0.5);
    }
    collector.extendCollectorCylinder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.collect(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
