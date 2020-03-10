/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

public class CmdMagazineHomeEncoder extends CommandBase {
  /**
   * Creates a new CmdMagazineHomeEncoder.
   */

  private final Magazine magazine;
  private double slowHomeSpeed = 0.1;

  public CmdMagazineHomeEncoder(Magazine magazine)
    {
      // Use addRequirements() here to declare subsystem dependencies.
      this.magazine = magazine;
      addRequirements(magazine);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
    {
      magazine.setHasHomed(false);
      System.out.println("CmdMagazineHomeEncoder.initialize");
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
    {  
      System.out.println("CmdMagazineHomeEncoder.execute");    
      magazine.setSpeed(slowHomeSpeed);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
    {
      magazine.setSpeed(0.0);
      magazine.zeroEncoder();
      System.out.println("Homed to shoot: " + magazine.getHomedToShootPosition());
      if(magazine.getHomedToShootPosition())
      {
        magazine.setSetpointDegrees(36.0);
      }
      else
      {
        magazine.setSetpointDegrees(0);
      }
      magazine.setHasHomed(true);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
    {      
      if(magazine.colorIsRed())
      {
        System.out.println("Red");
        magazine.setHomedToShootPosition(true);
        return true;
      }
      else
      {
        return false;
      }
    }
}
