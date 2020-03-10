/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LogitechGamePad;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Magazine;

public class CmdDefaultCollector extends CommandBase {
  /**
   * Creates a new CmdDefaultCollect.
   */
  private final Collector collector;
  private final Magazine magazine;
  private final Joystick driverJoystick;

  private boolean firstScanButtonPress = true;
  private boolean firstScanTimerOffset = true;
  private boolean firstScanOffsetMagazine = true;
  private boolean waitingForNextCollectPosition = true;
  private double delayTime = 0.0;

  public CmdDefaultCollector(Collector collector, Joystick driverJoystick, Magazine magazine) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.magazine = magazine;
    this.collector = collector;
    this.driverJoystick = driverJoystick;

    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driverJoystick.getRawAxis(LogitechGamePad.LEFT_TRIGGER) > 0.2)
    {
      if(firstScanButtonPress)
      {
        firstScanButtonPress = false;
        magazine.nextCollectIndex();
      }

      if(magazine.onTarget())
      {
        if(collector.isBallPresent())
        {
          magazine.nextCollectIndex();
          System.out.println("Ball at : " + magazine.getPositionInDegrees());
        }
        else
        {
          collector.extendCollectorCylinder();
          if(firstScanTimerOffset)
          {
            firstScanTimerOffset = false;
            delayTime = Timer.getFPGATimestamp() + 0.5;
          }

          if(Timer.getFPGATimestamp() > delayTime)
          {
            collector.collect(0.75, 0.6);
          }
        }
      }
      
      firstScanOffsetMagazine = true;
    }
    else
    {
      if(firstScanOffsetMagazine)
      {
        if(magazine.onTarget() && collector.isBallPresent())
        {
          magazine.nextShootIndex();
        }
        else
        {
          magazine.previousShootIndex();
        }
        firstScanOffsetMagazine = false;
      }
      firstScanButtonPress = true;
      firstScanTimerOffset = true;
      collector.collect(0.0, 0.0);
      collector.retractCollectorCylinder();
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
