/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControlPanel extends SubsystemBase {
  /**
   * Creates a new ControlPanel.
   */
  private final Joystick operatorJoystick;

  private final WPI_VictorSPX controlPanelVictorSPX = new WPI_VictorSPX(40);
  private final I2C.Port controlPanelColorSensorI2CPort = I2C.Port.kOnboard;
  private final ColorSensorV3 controlPanelColorSensor = new ColorSensorV3(controlPanelColorSensorI2CPort);

  private final ColorMatch colorMatcher = new ColorMatch();

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  int proximity;

  
  public ControlPanel(Joystick operatorJoystick) {
    this.operatorJoystick = operatorJoystick;
    
    proximity = controlPanelColorSensor .getProximity();
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);

  }

  @Override
  public void periodic() {
    Color detectedColor = controlPanelColorSensor.getColor();

    //System.out.println("Red" + detectedColor.red + "Green" + detectedColor.green + "Blue" + detectedColor.blue + "Distance" + proximity);
    //System.out.println("Most likely color : " + colorSensor.getColor().);
    /*ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    if(match.color == kBlueTarget)
    {
      System.out.println("Detected color : blue");
    }
    else if(match.color == kRedTarget)
    {
      System.out.println("Detected color : red");
    }
    else if(match.color == kGreenTarget)
    {
      System.out.println("Detected color : green");
    }
    else if(match.color == kYellowTarget)
    {
      System.out.println("Detected color : yellow");
    }
    else
    {
      System.out.println("Color unknown");
    }*/
  }
}
