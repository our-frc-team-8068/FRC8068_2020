/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.DriveTrain;

public class CmdControlPanelRotateTurns extends CommandBase {
  /**
   * Creates a new CmdRotateTurns.
   */
  private final ControlPanel controlPanel;
  private final DriveTrain driveTrain;
  private final ColorMatch controlPanelColorMatcher = new ColorMatch();

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  ColorMatchResult currentColor;
  Color targetColor;

  double endtime;

  int colorCount;
  int finalColorCount = 8;
  
  boolean allowColorCount = false;
  boolean onTargetColor;
  boolean offTargetColor;

  double debounceDuration = 0.06;
  double controlPanelMotorSpeed = 0.8;
  double onTargetColorEndTime;
  double offTargetColorEndTime;


  public CmdControlPanelRotateTurns(ControlPanel controlPanel, DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controlPanel = controlPanel;
    this.driveTrain = driveTrain;

    addRequirements(controlPanel, driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorCount = 0;

    controlPanelColorMatcher.addColorMatch(kBlueTarget);
    controlPanelColorMatcher.addColorMatch(kGreenTarget);
    controlPanelColorMatcher.addColorMatch(kRedTarget);
    controlPanelColorMatcher.addColorMatch(kYellowTarget);

    currentColor = controlPanelColorMatcher.matchClosestColor(controlPanel.getColorSensorColor());
    
    if (currentColor.color == kGreenTarget)
    {
      targetColor = kGreenTarget;
    }
    else if (currentColor.color == kYellowTarget)
    {
      targetColor = kYellowTarget;
    }
    else if (currentColor.color == kRedTarget)
    {
      targetColor = kRedTarget;
    }
    else
    {
      targetColor = kBlueTarget;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Initial Target Color " + targetColor);
    controlPanel.setControlPanelSpeed(controlPanelMotorSpeed);
    currentColor = controlPanelColorMatcher.matchClosestColor(controlPanel.getColorSensorColor());
    //System.out.println("Color Counts " + colorCount);
    System.out.println("OnColorTarget " + onTargetColor);

    //Debouncing color states
    if(currentColor.color != targetColor) //Debouncing color match state
    {
      onTargetColorEndTime = Timer.getFPGATimestamp() + debounceDuration;
    }
    else if(currentColor.color == targetColor)  //Debouncing color mismatch state
    {
      offTargetColorEndTime = Timer.getFPGATimestamp() + debounceDuration;
    }

    //Determining color match state
    if(Timer.getFPGATimestamp() > onTargetColorEndTime)
    {
      if(!onTargetColor)
      {
        allowColorCount = true;
      }
      onTargetColor = true;

    }
    else if(Timer.getFPGATimestamp() > offTargetColorEndTime)
    {
      onTargetColor = false;

    }
    
    if(onTargetColor && allowColorCount)
    {
      colorCount++;
      allowColorCount = false;
    }

    /*if(currentColor.color == targetColor)
    {
    }
    else if(Timer.getFPGATimestamp() > offTargetColorEndTime)
    {
    }
    
    if (currentColor.color != targetColor) //Might need to change back to !=
    {
      endtime = Timer.getFPGATimestamp() + verificationTimeoutTime;
      allowColorCount = true;
    }
    else
    {
      if(Timer.getFPGATimestamp() > endtime)
      {
        if(allowColorCount)
        {
          colorCount++;
          allowColorCount = false;
        }
      }
      controlPanel.setControlPanelSpeed(controlPanelMotorSpeed);
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controlPanel.setControlPanelSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Color Count " + colorCount);
    if (colorCount >= finalColorCount)
    {
      return true;
    }
    else 
    {
    return false;
    }
  }
}
