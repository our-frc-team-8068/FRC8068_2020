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

  private final ColorMatch magazineColorMatcher = new ColorMatch();

  ColorMatchResult match;

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
      magazineColorMatcher.addColorMatch(magazine.getkRedTarget());
      magazineColorMatcher.addColorMatch(magazine.getkGreenTarget());
      magazineColorMatcher.addColorMatch(magazine.getkBlueTarget());
      magazineColorMatcher.addColorMatch(magazine.getkYellowTarget());
      magazine.setHasHomed(false);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
    {
      match = magazineColorMatcher.matchClosestColor(magazine.getColorSensorColor()); 
      
      if(match.color == magazine.getkBlueTarget())
      {
        magazine.setMagazineSpeed(0.05);
        System.out.println("Blue");
      }
      else if(match.color == magazine.getkGreenTarget())
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
      if(match.color == magazine.getkRedTarget() || match.color == magazine.getkYellowTarget())
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
