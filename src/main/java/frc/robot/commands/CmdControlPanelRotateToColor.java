/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.DriveTrain;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;


public class CmdControlPanelRotateToColor extends CommandBase {
  /**
   * Creates a new CmdControlPanelRotateToColor.
   */
  private final ControlPanel controlPanel;
  private final DriveTrain driveTrain;
  private final ColorMatch controlPanelColorMatcher = new ColorMatch();
  
  String gameData = DriverStation.getInstance().getGameSpecificMessage();
  
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  ColorMatchResult match;

  Timer timer = new Timer();

  double endtime;

  boolean firstScan = true;

  double verificationTimeoutTime = 5.0;

  public CmdControlPanelRotateToColor(ControlPanel controlPanel, DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controlPanel = controlPanel;
    this.driveTrain = driveTrain;
    
    addRequirements(controlPanel, driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controlPanelColorMatcher.addColorMatch(kBlueTarget);
    controlPanelColorMatcher.addColorMatch(kGreenTarget);
    controlPanelColorMatcher.addColorMatch(kRedTarget);
    controlPanelColorMatcher.addColorMatch(kYellowTarget);
    
    System.out.println("Game Data is " + gameData.charAt(0));
    /*if(gameData.length() > 0)
    {
      if (gameData.charAt(0) == 'Y')
      {
        System.out.println("Set Color to Green");
      }
      else if (gameData.charAt(0) == 'G')
      {
        System.out.println("Set Color to Yellow");
      }
      else if (gameData.charAt(0) == 'B')
      {
        System.out.println("Set Color to Red");
      }
      else if (gameData.charAt(0) == 'R')
      {
        System.out.println("Set Color to BLue");
      }
      else
      {
        System.out.println("Game Data not Set");
      }
    }*/
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controlPanel.setControlPanelSpeed(0.15);
    match = controlPanelColorMatcher.matchClosestColor(controlPanel.getColorSensorColor()); 
    System.out.println("Color Confidence" + match.confidence);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    firstScan = true; // Reset the first scan boolean back to true
    controlPanel.setControlPanelSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(gameData.length() > 0)
    {
      if (gameData.charAt(0) == 'Y' && match.color == kGreenTarget)
      {
        if (firstScan)
        {
          endtime = timer.getFPGATimestamp() + verificationTimeoutTime;
          firstScan = false;
        }

        System.out.println("We are waiting for the verification within Yellow");

        if (timer.getFPGATimestamp() > endtime) 
        {
          System.out.println("Set Color to Green");
          return true;
        }
        else
        {
          return false;
        }
      }
      else if (gameData.charAt(0) == 'G' && match.color == kYellowTarget)
      {
        if (firstScan)
        {
          endtime = timer.getFPGATimestamp() + verificationTimeoutTime;
          firstScan = false;
        }
        if (timer.getFPGATimestamp() > endtime) 
        {
          System.out.println("Set Color to Yellow");
          return true;
        }
        else
        {
          return false;
        }
      }
      else if (gameData.charAt(0) == 'B' && match.color == kRedTarget)
      {
        if (firstScan)
        {
          endtime = timer.getFPGATimestamp() + verificationTimeoutTime;
          firstScan = false;
        }
        if (timer.getFPGATimestamp() > endtime) 
        {
          System.out.println("Set Color to Red");
          return true;
        }
        else
        {
          return false;
        }
      }
      else if (gameData.charAt(0) == 'R' && match.color == kBlueTarget)
      {
        if (firstScan)
        {
          endtime = timer.getFPGATimestamp() + verificationTimeoutTime;
          firstScan = false;
        }
        if (timer.getFPGATimestamp() > endtime) 
        {
          System.out.println("Set Color to Blue");
          return true;
        }
        else
        {
          return false;
        }
      }
      else
      {
        return false;
      }
    }
    else
    {
      System.out.println("Game Data not Set");
      return false;
    }

  }
}
