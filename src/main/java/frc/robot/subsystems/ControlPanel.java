/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControlPanel extends SubsystemBase {
  /**
   * Creates a new ControlPanel.
   */
  private final Joystick operatorJoystick;

  private final WPI_VictorSPX controlPanelVictorSPX = new WPI_VictorSPX(40);
  private final I2C.Port controlPanelColorSensorI2CPort = I2C.Port.kOnboard; //This should be MXP but we changed it to work on QP
  private final ColorSensorV3 controlPanelColorSensor = new ColorSensorV3(controlPanelColorSensorI2CPort);

  private final ColorMatch colorMatcher = new ColorMatch();
  
  private double kBlueTargetRedValue = 0.0;
  private double kBlueTargetGreenValue = 0.0;
  private double kBlueTargetBlueValue = 0.0;

  private double kGreenTargetRedValue = 0.0;
  private double kGreenTargetGreenValue = 0.0;
  private double kGreenTargetBlueValue = 0.0;

  private double kRedTargetRedValue = 0.0;
  private double kRedTargetGreenValue = 0.0;
  private double kRedTargetBlueValue = 0.0;

  private double kYellowTargetRedValue = 0.0;
  private double kYellowTargetGreenValue = 0.0;
  private double kYellowTargetBlueValue = 0.0;

  private final Color kBlueTarget = ColorMatch.makeColor(kBlueTargetRedValue, kBlueTargetGreenValue, kBlueTargetBlueValue);
  private final Color kGreenTarget = ColorMatch.makeColor(kGreenTargetRedValue, kGreenTargetGreenValue, kGreenTargetBlueValue);
  private final Color kRedTarget = ColorMatch.makeColor(kRedTargetRedValue, kRedTargetGreenValue, kRedTargetBlueValue);
  private final Color kYellowTarget = ColorMatch.makeColor(kYellowTargetRedValue, kYellowTargetGreenValue, kYellowTargetBlueValue);
  
//  private ShuffleboardTab magazineTab = Shuffleboard.getTab("Magazine"); 
//private NetworkTableEntry ntSTSmagazineSetpointDegrees = 
//  magazineTab.add("STSMagazineSetPointDegrees", setpointDegrees).withSize(2, 1).withPosition(0, 0).getEntry();

  //int proximity;
  
  boolean getRedColorCalibration = false;
  boolean getGreenColorCalibration = false;
  boolean getBlueColorCalibration = false;
  boolean getYellowColorCalibration = false;


  private ShuffleboardTab controlPanelTab = Shuffleboard.getTab("Control Panel");
  private NetworkTableEntry ntControlPanelRedColorCalibration =
    controlPanelTab.add("ControlPanelRedColorCalibration", getRedColorCalibration)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(0, 0).getEntry();     
  private NetworkTableEntry ntControlPanelGreenColorCalibration =
    controlPanelTab.add("ControlPanelGreenColorCalibration", getGreenColorCalibration)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(2, 0).getEntry();    
  private NetworkTableEntry ntControlPanelBlueColorCalibration =
    controlPanelTab.add("ControlPanelBlueColorCalibration", getBlueColorCalibration)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(4, 0).getEntry();
  private NetworkTableEntry ntControlPanelYellowColorCalibration =
    controlPanelTab.add("ControlPanelYellowColorCalibration", getYellowColorCalibration)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(6, 0).getEntry();

  private NetworkTableEntry ntControlPanelRedRValue =
    controlPanelTab.addPersistent("Red R Value", kRedTargetRedValue).withSize(2, 1).withPosition(0, 1).getEntry();
  private NetworkTableEntry ntControlPanelRedGValue =
    controlPanelTab.addPersistent("Red G Value", kRedTargetGreenValue).withSize(2, 1).withPosition(0, 2).getEntry();
  private NetworkTableEntry ntControlPanelRedBValue =
    controlPanelTab.addPersistent("Red B Value", kRedTargetBlueValue).withSize(2, 1).withPosition(0, 3).getEntry();

  private NetworkTableEntry ntControlPanelGreenRValue =
    controlPanelTab.addPersistent("Green R Value", kGreenTargetRedValue).withSize(2, 1).withPosition(2, 1).getEntry();
  private NetworkTableEntry ntControlPanelGreenGValue =
    controlPanelTab.addPersistent("Green G Value", kGreenTargetGreenValue).withSize(2, 1).withPosition(2, 2).getEntry();
  private NetworkTableEntry ntControlPanelGreenBValue =
    controlPanelTab.addPersistent("Green B Value", kGreenTargetBlueValue).withSize(2, 1).withPosition(2, 3).getEntry();

  private NetworkTableEntry ntControlPanelBlueRValue =
    controlPanelTab.addPersistent("Blue R Value", kBlueTargetRedValue).withSize(2, 1).withPosition(4, 1).getEntry();
  private NetworkTableEntry ntControlPanelBlueGValue =
    controlPanelTab.addPersistent("Blue G Value", kBlueTargetGreenValue).withSize(2, 1).withPosition(4, 2).getEntry();
  private NetworkTableEntry ntControlPanelBlueBValue =
    controlPanelTab.addPersistent("Blue B Value", kBlueTargetBlueValue).withSize(2, 1).withPosition(4, 3).getEntry();

  private NetworkTableEntry ntControlPanelYellowRValue =
    controlPanelTab.addPersistent("Yellow R Value", kYellowTargetRedValue).withSize(2, 1).withPosition(6, 1).getEntry();
  private NetworkTableEntry ntControlPanelYellowGValue =
    controlPanelTab.addPersistent("Yellow G Value", kYellowTargetGreenValue).withSize(2, 1).withPosition(6, 2).getEntry();
  private NetworkTableEntry ntControlPanelYellowBValue =
    controlPanelTab.addPersistent("Yellow B Value", kYellowTargetBlueValue).withSize(2, 1).withPosition(6, 3).getEntry();
    
  public ControlPanel(Joystick operatorJoystick) {
    this.operatorJoystick = operatorJoystick;
    
    //proximity = controlPanelColorSensor .getProximity();
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);

  }

  @Override
  public void periodic() {
    Color detectedColor = controlPanelColorSensor.getColor();
    System.out.println("Blue Color Value " + kBlueTarget);
    System.out.println("Boolean State " + getBlueColorCalibration);
    System.out.println("Blue R Value " + kBlueTargetRedValue + 
      " Blue Green Value " + kBlueTargetGreenValue + " Blue Blue Value " + kBlueTargetBlueValue);

    kRedTargetRedValue = ntControlPanelRedRValue.getDouble(kRedTargetRedValue);
    kRedTargetGreenValue = ntControlPanelRedGValue.getDouble(kRedTargetGreenValue);
    kRedTargetBlueValue = ntControlPanelRedBValue.getDouble(kRedTargetBlueValue);

    kGreenTargetRedValue = ntControlPanelGreenRValue.getDouble(kGreenTargetRedValue);
    kGreenTargetGreenValue = ntControlPanelGreenGValue.getDouble(kGreenTargetGreenValue);
    kGreenTargetBlueValue = ntControlPanelGreenBValue.getDouble(kGreenTargetBlueValue);

    kBlueTargetRedValue = ntControlPanelBlueRValue.getDouble(kBlueTargetRedValue);
    kBlueTargetGreenValue = ntControlPanelBlueGValue.getDouble(kBlueTargetGreenValue);
    kBlueTargetBlueValue = ntControlPanelBlueBValue.getDouble(kBlueTargetBlueValue);

    kYellowTargetRedValue = ntControlPanelYellowRValue.getDouble(kYellowTargetRedValue);
    kYellowTargetGreenValue = ntControlPanelYellowGValue.getDouble(kYellowTargetGreenValue);
    kYellowTargetBlueValue = ntControlPanelYellowBValue.getDouble(kYellowTargetBlueValue);

    if(ntControlPanelRedColorCalibration.getBoolean(false))
    {
      kRedTargetRedValue = detectedColor.red;
      kRedTargetGreenValue = detectedColor.green;
      kRedTargetBlueValue = detectedColor.blue;

      ntControlPanelRedRValue.forceSetDouble(kRedTargetRedValue);
      ntControlPanelRedGValue.forceSetDouble(kRedTargetGreenValue);
      ntControlPanelRedBValue.forceSetDouble(kRedTargetBlueValue);

      ntControlPanelRedColorCalibration.setBoolean(false);
    }
    
    if(ntControlPanelGreenColorCalibration.getBoolean(false))
    {
      kGreenTargetRedValue = detectedColor.red;
      kGreenTargetGreenValue = detectedColor.green;
      kGreenTargetBlueValue = detectedColor.blue; 

      ntControlPanelGreenRValue.forceSetDouble(kGreenTargetRedValue);
      ntControlPanelGreenGValue.forceSetDouble(kGreenTargetGreenValue);
      ntControlPanelGreenBValue.forceSetDouble(kGreenTargetBlueValue);

      ntControlPanelGreenColorCalibration.setBoolean(false);
    }

    if(ntControlPanelBlueColorCalibration.getBoolean(false))
    {
      kBlueTargetRedValue = detectedColor.red;
      kBlueTargetGreenValue = detectedColor.green;
      kBlueTargetBlueValue = detectedColor.blue;

      ntControlPanelBlueRValue.forceSetDouble(kBlueTargetRedValue);
      ntControlPanelBlueGValue.forceSetDouble(kBlueTargetGreenValue);
      ntControlPanelBlueBValue.forceSetDouble(kBlueTargetBlueValue);

      ntControlPanelBlueColorCalibration.setBoolean(false);
    }

    if(ntControlPanelYellowColorCalibration.getBoolean(false))
    {
      kYellowTargetRedValue = detectedColor.red;
      kYellowTargetGreenValue = detectedColor.green;
      kYellowTargetBlueValue = detectedColor.blue;

      ntControlPanelYellowRValue.forceSetDouble(kYellowTargetRedValue);
      ntControlPanelYellowGValue.forceSetDouble(kYellowTargetGreenValue);
      ntControlPanelYellowBValue.forceSetDouble(kYellowTargetBlueValue);
      
      ntControlPanelYellowColorCalibration.setBoolean(false);
    }

    //System.out.println("Red" + detectedColor.red + "Green" + detectedColor.green + "Blue" + detectedColor.blue + " Raw BLue is " + controlPanelColorSensor.getBlue());
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

  public void setControlPanelSpeed(double speed){
    controlPanelVictorSPX.setNeutralMode(NeutralMode.Brake);
    controlPanelVictorSPX.set(ControlMode.PercentOutput, speed);
  } 

  public Color getColorSensorColor()
  {
    return controlPanelColorSensor.getColor();
  }
  
}
