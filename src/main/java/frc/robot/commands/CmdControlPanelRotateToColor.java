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
  
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  ColorMatchResult colorMatchResult;
  Color targetColor;

  double endtime;

  boolean firstScan = true;
  boolean invertDirection = false;

  double verificationTimeoutTime = 1.0;
  double controlPanelMotorSpeed = 0.15;

  public CmdControlPanelRotateToColor(ControlPanel controlPanel, DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controlPanel = controlPanel;
    this.driveTrain = driveTrain;
    
    addRequirements(controlPanel, driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    String gameData = DriverStation.getInstance().getGameSpecificMessage();

    controlPanelColorMatcher.addColorMatch(kBlueTarget);
    controlPanelColorMatcher.addColorMatch(kGreenTarget);
    controlPanelColorMatcher.addColorMatch(kRedTarget);
    controlPanelColorMatcher.addColorMatch(kYellowTarget);

    colorMatchResult = controlPanelColorMatcher.matchClosestColor(controlPanel.getColorSensorColor());
    if(gameData.length() > 0)
    {
      if (gameData.charAt(0) == 'Y')
      {
        targetColor = kGreenTarget;
        if(colorMatchResult.color == kBlueTarget)
        {
          invertDirection = true;
        }
      }
      else if (gameData.charAt(0) == 'G')
      {
        targetColor = kYellowTarget;
        if(colorMatchResult.color == kRedTarget)
        {
          invertDirection = true;
        }
      }
      else if (gameData.charAt(0) == 'B')
      {
        targetColor = kRedTarget;
        if(colorMatchResult.color == kGreenTarget)
        {
          invertDirection = true;
        }
      }
      else if (gameData.charAt(0) == 'R')
      {
        targetColor = kBlueTarget;
        if(colorMatchResult.color == kYellowTarget)
        {
          invertDirection = true;
        }
      }
      else
      {
        System.out.println("Game Data not Set");
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    colorMatchResult = controlPanelColorMatcher.matchClosestColor(controlPanel.getColorSensorColor());
    if (targetColor != colorMatchResult.color) 
    {
      endtime = Timer.getFPGATimestamp() + verificationTimeoutTime;
      if(invertDirection)
      {
        controlPanel.setControlPanelSpeed(-controlPanelMotorSpeed);
      }
      else
      {
        controlPanel.setControlPanelSpeed(controlPanelMotorSpeed);
      }
    }
    else
    {
      controlPanel.setControlPanelSpeed(0.0);
    }
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
    if(Timer.getFPGATimestamp() > endtime)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
