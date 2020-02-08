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

  private final ColorMatch colorMatcher = new ColorMatch();

  ColorMatchResult match;

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

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
      colorMatcher.addColorMatch(kBlueTarget);
      colorMatcher.addColorMatch(kGreenTarget);
      colorMatcher.addColorMatch(kRedTarget);
      colorMatcher.addColorMatch(kYellowTarget);
      magazine.setHasHomed(false);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
    {
      match = colorMatcher.matchClosestColor(magazine.getColorSensorColor()); 
      
      if(match.color == kBlueTarget)
      {
        magazine.setMagazineSpeed(0.05);
        System.out.println("Blue");
      }
      else if(match.color == kGreenTarget)
      {
        magazine.setMagazineSpeed(-0.05);
        System.out.println("Green");
      }
      else //This may need to be set specifically to only go to 100% if it detects black
      {
        magazine.setMagazineSpeed(0.1);
      }
      System.out.println("Has Homed " + magazine.getHasHomed());
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
    {
      magazine.setMagazineSpeed(0.0);
      magazine.zeroEncoder();
      magazine.setSetpointDegrees(0);
      magazine.setHasHomed(true);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
    {      
      if(match.color == kRedTarget || match.color == kYellowTarget)
      {
        System.out.println("Red or Yellow");
        return true;
      }
      else
      {
        return false;
      }
    }
}
