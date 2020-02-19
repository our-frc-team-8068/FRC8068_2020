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

  Color targetColor;

  double endtime;

  char manuallyDesignatedColor;

  boolean useFMSColor;
  boolean firstScan = true;
  boolean invertDirection = false;

  double verificationTimeoutTime = 1.0;
  double controlPanelMotorSpeed = 0.15;

  public CmdControlPanelRotateToColor(ControlPanel controlPanel, DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controlPanel = controlPanel;
    this.driveTrain = driveTrain;

    useFMSColor = true;
    
    addRequirements(controlPanel, driveTrain);
  }

  public CmdControlPanelRotateToColor(ControlPanel controlPanel, DriveTrain driveTrain, char manuallyDesignatedColor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controlPanel = controlPanel;
    this.driveTrain = driveTrain;
    this.manuallyDesignatedColor = manuallyDesignatedColor;

    useFMSColor = false;
    
    addRequirements(controlPanel, driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    char targetColorChar;

    if(useFMSColor)
    {
      
      if(gameData.length() > 0)
      {
        targetColorChar = gameData.charAt(0);
      }
      else
      {
        targetColorChar = 'x';
      }
      
    }
    else
    {
      targetColorChar = manuallyDesignatedColor;
    }

    if (targetColorChar == 'Y')
    {
      targetColor = controlPanel.getKGreenTarget();
      if(controlPanel.colorIsBlue())
      {
        invertDirection = true;
      }
    }
    else if (targetColorChar == 'G')
    {
      targetColor = controlPanel.getKYellowTarget();
      if(controlPanel.colorIsRed())
      {
        invertDirection = true;
      }
    }
    else if (targetColorChar == 'B')
    {
      targetColor = controlPanel.getKRedTarget();
      if(controlPanel.colorIsGreen())
      {
        invertDirection = true;
      }
    }
    else if (targetColorChar == 'R')
    {
      targetColor = controlPanel.getKBlueTarget();
      if(controlPanel.colorIsYellow())
      {
        invertDirection = true;
      }
    }
    else
    {
      System.out.println("Game Data not Set");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    if (targetColor != controlPanel.getColorMatcherColor()) 
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
    controlPanel.setControlPanelSpeed(0.0);
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
