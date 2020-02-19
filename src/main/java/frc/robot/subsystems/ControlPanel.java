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
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CmdControlPanelRotateToColor;
import frc.robot.commands.CmdControlPanelRotateTurns;

public class ControlPanel extends SubsystemBase {
  /**
   * Creates a new ControlPanel.
   */
  private final Joystick operatorJoystick;

  private final WPI_VictorSPX controlPanelVictorSPX = new WPI_VictorSPX(40);    //40
  private final I2C.Port controlPanelColorSensorI2CPort = I2C.Port.kOnboard; //This should be MXP but we changed it to work on QP
  private final ColorSensorV3 colorSensor = new ColorSensorV3(controlPanelColorSensorI2CPort);
  private boolean colorCalibrationEnabled = false;
  private ShuffleboardTab controlPanelColorTab = Shuffleboard.getTab("ControlPanelColor");
  private ShuffleboardTab controlPanelControlTab = Shuffleboard.getTab("ControlPanelControl").;
  private double numberOfTurnsToRotate = 0;

  private ComplexWidget ntCmdRotateTurns;
  private ComplexWidget ntCmdRotateToRed;
  private ComplexWidget ntCmdRotateToGreen;
  private ComplexWidget ntCmdRotateToBlue;
  private ComplexWidget ntCmdRotateToYellow;
  private NetworkTableEntry ntScdNumberOfTurnsToRotate = controlPanelControlTab.add("SCD Number Of Turns To Rotate", 69.0)
    .withSize(2, 1).withPosition(0, 0).getEntry();
  private NetworkTableEntry ntStsNumberOfTurnsToRotate = controlPanelControlTab.addPersistent("STS Number Of Turns To Rotate", 69.0)
    .withSize(2, 1).withPosition(4, 0).getEntry();

  private NetworkTableEntry ntSTSNumberOfColorCounts = controlPanelControlTab.add("STS Number of Color Counts", 69.0)
    .withSize(2, 1).withPosition(6, 0).getEntry();

  private ColorMatch colorMatcher = new ColorMatch();

  private ColorMatchResult colorMatchResult;
  
  private boolean colorIsRed = false;
  private boolean colorIsGreen = false;
  private boolean colorIsBlue = false;
  private boolean colorIsYellow = false;

  private double kRedTargetRedValue;
  private double kRedTargetGreenValue;
  private double kRedTargetBlueValue;
  private NetworkTableEntry ntRedRValue =
    controlPanelColorTab.addPersistent("controlPanel Red R Value", 69.0).withSize(2, 1).withPosition(2, 1).getEntry();
  private NetworkTableEntry ntRedGValue =
    controlPanelColorTab.addPersistent("controlPanel Red G Value", 69.0).withSize(2, 1).withPosition(4, 1).getEntry();
  private NetworkTableEntry ntRedBValue =
    controlPanelColorTab.addPersistent("controlPanel Red B Value", 69.0).withSize(2, 1).withPosition(6, 1).getEntry();
  private Color kRedTarget;

  private double kGreenTargetRedValue;
  private double kGreenTargetGreenValue;
  private double kGreenTargetBlueValue;
  private NetworkTableEntry ntGreenRValue =
    controlPanelColorTab.addPersistent("controlPanel Green R Value", 69.0).withSize(2, 1).withPosition(2, 2).getEntry();
  private NetworkTableEntry ntGreenGValue =
    controlPanelColorTab.addPersistent("controlPanel Green G Value", 69.0).withSize(2, 1).withPosition(4, 2).getEntry();
  private NetworkTableEntry ntGreenBValue =
    controlPanelColorTab.addPersistent("controlPanel Green B Value", 69.0).withSize(2, 1).withPosition(6, 2).getEntry();
  private Color kGreenTarget;

  private double kBlueTargetRedValue;
  private double kBlueTargetGreenValue;
  private double kBlueTargetBlueValue;
  private NetworkTableEntry ntBlueRValue =
    controlPanelColorTab.addPersistent("controlPanel Blue R Value", 69.0).withSize(2, 1).withPosition(2, 3).getEntry();
  private NetworkTableEntry ntBlueGValue =
    controlPanelColorTab.addPersistent("controlPanel Blue G Value", 69.0).withSize(2, 1).withPosition(4, 3).getEntry();
  private NetworkTableEntry ntBlueBValue =
    controlPanelColorTab.addPersistent("controlPanel Blue B Value", 69.0).withSize(2, 1).withPosition(6, 3).getEntry();
  private Color kBlueTarget;

  private double kYellowTargetRedValue;
  private double kYellowTargetGreenValue;
  private double kYellowTargetBlueValue;
  private NetworkTableEntry ntYellowRValue =
    controlPanelColorTab.addPersistent("controlPanel Yellow R Value", 69.0).withSize(2, 1).withPosition(2, 4).getEntry();
  private NetworkTableEntry ntYellowGValue =
    controlPanelColorTab.addPersistent("controlPanel Yellow G Value", 69.0).withSize(2, 1).withPosition(4, 4).getEntry();
  private NetworkTableEntry ntYellowBValue =
    controlPanelColorTab.addPersistent("controlPanel Yellow B Value", 69.0).withSize(2, 1).withPosition(6, 4).getEntry();
  private Color kYellowTarget;

  boolean getRedColorCalibration = false;
  boolean getGreenColorCalibration = false;
  boolean getBlueColorCalibration = false;
  boolean getYellowColorCalibration = false;

  private NetworkTableEntry ntRedColorCalibration =
    controlPanelColorTab.add("controlPanel Red Color Calibration", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(0, 1).getEntry();     
  private NetworkTableEntry ntGreenColorCalibration =
    controlPanelColorTab.add("controlPanel Green Color Calibration", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(0, 2).getEntry();    
  private NetworkTableEntry ntBlueColorCalibration =
    controlPanelColorTab.add("controlPanel Blue Color Calibration", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(0, 3).getEntry();
  private NetworkTableEntry ntYellowColorCalibration =
    controlPanelColorTab.add("controlPanel Yellow Color Calibration", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(0, 4).getEntry();

  private NetworkTableEntry ntColorIsRed = controlPanelColorTab.add("Is Red", colorIsRed).withSize(1, 1).withPosition(8, 1).getEntry();
  private NetworkTableEntry ntColorIsGreen = controlPanelColorTab.add("Is Green", colorIsGreen).withSize(1, 1).withPosition(8, 2).getEntry();
  private NetworkTableEntry ntColorIsBlue = controlPanelColorTab.add("Is Blue", colorIsBlue).withSize(1, 1).withPosition(8, 3).getEntry();
  private NetworkTableEntry ntColorIsYellow = controlPanelColorTab.add("Is Yellow", colorIsYellow).withSize(1, 1).withPosition(8, 4).getEntry();

  private NetworkTableEntry ntColorCalibrationEnabled = controlPanelColorTab.add("controlPanel Color Calibration Enabled", colorCalibrationEnabled)
    .withWidget(BuiltInWidgets.kToggleSwitch).withSize(2, 1).withPosition(0, 0).getEntry();

  private NetworkTableEntry ntControlPanelColorSensorCurrentRed = controlPanelColorTab.add("controlPanelColorSensorCurrentRed", 69.0).withSize(2,1).withPosition(2,0).getEntry();
  private NetworkTableEntry ntControlPanelColorSensorCurrentGreen = controlPanelColorTab.add("controlPanelColorSensorCurentGreen", 69.0).withSize(2, 1).withPosition(4, 0).getEntry();
  private NetworkTableEntry ntControlPanelColorSensorCurrentBlue = controlPanelColorTab.add("controlPanelColorSensorCurrentBlue", 69.0).withSize(2, 1).withPosition(6, 0).getEntry();
    
  public ControlPanel(Joystick operatorJoystick, DriveTrain driveTrain) {
    this.operatorJoystick = operatorJoystick;
    getNewShuffleboardData();

    kRedTarget = new Color(new Color8Bit((int) (kRedTargetRedValue * 255), (int) (kRedTargetGreenValue * 255), (int) (kRedTargetBlueValue * 255)));
    kGreenTarget  = new Color(new Color8Bit((int) (kGreenTargetRedValue * 255), (int) (kGreenTargetGreenValue * 255), (int) (kGreenTargetBlueValue * 255)));
    kBlueTarget = new Color(new Color8Bit((int) (kBlueTargetRedValue * 255), (int) (kBlueTargetGreenValue * 255), (int) (kBlueTargetBlueValue * 255)));
    kYellowTarget  = new Color(new Color8Bit((int) (kYellowTargetRedValue * 255), (int) (kYellowTargetGreenValue * 255), (int) (kYellowTargetBlueValue * 255)));

    updateColorMatcher();
    
    //proximity = controlPanelColorSensor .getProximity();
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);

    ntCmdRotateTurns = controlPanelControlTab.add("CmdRotateTurns", new CmdControlPanelRotateTurns(this, driveTrain)).withSize(2, 1).withPosition(2, 0);

    ntCmdRotateToRed = controlPanelControlTab.add("CmdRotateToRed", new CmdControlPanelRotateToColor(this, driveTrain, 'R')).withSize(2, 1).withPosition(0, 1);

    ntCmdRotateToGreen = controlPanelControlTab.add("CmdRotateToGreen", new CmdControlPanelRotateToColor(this, driveTrain, 'G')).withSize(2, 1).withPosition(2, 1);

    ntCmdRotateToBlue = controlPanelControlTab.add("CmdRotateToBlue", new CmdControlPanelRotateToColor(this, driveTrain, 'B')).withSize(2, 1).withPosition(4, 1);

    ntCmdRotateToYellow = controlPanelControlTab.add("CmdRotateToYellow", new CmdControlPanelRotateToColor(this, driveTrain, 'Y')).withSize(2, 1).withPosition(6, 1);
  }

  @Override
  public void periodic() {
    ntControlPanelColorSensorCurrentRed.forceSetDouble(colorSensor.getColor().red);
    ntControlPanelColorSensorCurrentGreen.forceSetDouble(colorSensor.getColor().green);
    ntControlPanelColorSensorCurrentBlue.forceSetDouble(colorSensor.getColor().blue);

    colorCalibrationEnabled = ntColorCalibrationEnabled.getBoolean(false);

    colorMatchResult = colorMatcher.matchClosestColor(colorSensor.getColor()); 

    if(colorMatchResult.color == kRedTarget)
    {
      colorIsRed = true;
      colorIsGreen = false;
      colorIsBlue = false;
      colorIsYellow = false;
    }
    else if(colorMatchResult.color == kGreenTarget)
    {
      colorIsRed = false;
      colorIsGreen = true;
      colorIsBlue = false;
      colorIsYellow = false;
    }
    else if(colorMatchResult.color == kBlueTarget)
    {
      colorIsRed = false;
      colorIsGreen = false;
      colorIsBlue = true;
      colorIsYellow = false;
    }
    else if(colorMatchResult.color == kYellowTarget)
    {
      colorIsRed = false;
      colorIsGreen = false;
      colorIsBlue = false;
      colorIsYellow = true;
    }

    ntColorIsRed.setBoolean(colorIsRed);
    ntColorIsGreen.setBoolean(colorIsGreen);
    ntColorIsBlue.setBoolean(colorIsBlue);
    ntColorIsYellow.setBoolean(colorIsYellow);

    colorCalibration();

    numberOfTurnsToRotate = ntScdNumberOfTurnsToRotate.getDouble(69.0);
    ntStsNumberOfTurnsToRotate.forceSetDouble(numberOfTurnsToRotate);
  }

  public double getNumberOfTurnsToRotate()
  {
    return numberOfTurnsToRotate;
  }

  public void setControlPanelSpeed(double speed){
    controlPanelVictorSPX.setNeutralMode(NeutralMode.Brake);
    controlPanelVictorSPX.set(ControlMode.PercentOutput, speed);
  } 

  public Color getColorSensorColor()
  {
    return colorSensor.getColor();
  }
  
  private void getNewShuffleboardData()
  {
    kRedTargetRedValue = ntRedRValue.getDouble(69.0);
    kRedTargetGreenValue = ntRedGValue.getDouble(69.0);
    kRedTargetBlueValue = ntRedBValue.getDouble(69.0);

    kGreenTargetRedValue = ntGreenRValue.getDouble(69.0);
    kGreenTargetGreenValue = ntGreenGValue.getDouble(69.0);
    kGreenTargetBlueValue = ntGreenBValue.getDouble(69.0);

    kBlueTargetRedValue = ntBlueRValue.getDouble(69.0);
    kBlueTargetGreenValue = ntBlueGValue.getDouble(69.0);
    kBlueTargetBlueValue = ntBlueBValue.getDouble(69.0);

    kYellowTargetRedValue = ntYellowRValue.getDouble(69.0);
    kYellowTargetGreenValue = ntYellowGValue.getDouble(69.0);
    kYellowTargetBlueValue = ntYellowBValue.getDouble(69.0);

  }

  private void updateColorMatcher()
  {
    colorMatcher = new ColorMatch();
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kYellowTarget);
  }

  private void colorCalibration()
  {
    Color detectedColor = colorSensor.getColor();

    if(ntRedColorCalibration.getBoolean(false))
    {
      if(colorCalibrationEnabled)
      {
        kRedTargetRedValue = detectedColor.red;
        kRedTargetGreenValue = detectedColor.green;
        kRedTargetBlueValue = detectedColor.blue;

        ntRedRValue.forceSetDouble(kRedTargetRedValue);
        ntRedGValue.forceSetDouble(kRedTargetGreenValue);
        ntRedBValue.forceSetDouble(kRedTargetBlueValue);
        kRedTarget = new Color(new Color8Bit((int) (kRedTargetRedValue * 255), 
          (int) (kRedTargetGreenValue * 255), (int) (kRedTargetBlueValue * 255)));

        updateColorMatcher();
      }
      colorCalibrationEnabled = false;
      ntRedColorCalibration.setBoolean(false);
    }
    
    if(ntGreenColorCalibration.getBoolean(false))
    {
      if(colorCalibrationEnabled)
      {
        kGreenTargetRedValue = detectedColor.red;
        kGreenTargetGreenValue = detectedColor.green;
        kGreenTargetBlueValue = detectedColor.blue; 

        ntGreenRValue.forceSetDouble(kGreenTargetRedValue);
        ntGreenGValue.forceSetDouble(kGreenTargetGreenValue);
        ntGreenBValue.forceSetDouble(kGreenTargetBlueValue);
        kGreenTarget = new Color(new Color8Bit((int) (kGreenTargetRedValue * 255), 
          (int) (kGreenTargetGreenValue * 255), (int) (kGreenTargetBlueValue * 255)));

        updateColorMatcher();
      }
      colorCalibrationEnabled = false;
      ntGreenColorCalibration.setBoolean(false);
    }

    if(ntBlueColorCalibration.getBoolean(false))
    {
      if(colorCalibrationEnabled)
      {
        kBlueTargetRedValue = detectedColor.red;
        kBlueTargetGreenValue = detectedColor.green;
        kBlueTargetBlueValue = detectedColor.blue;

        ntBlueRValue.forceSetDouble(kBlueTargetRedValue);
        ntBlueGValue.forceSetDouble(kBlueTargetGreenValue);
        ntBlueBValue.forceSetDouble(kBlueTargetBlueValue);
        kBlueTarget = new Color(new Color8Bit((int) (kBlueTargetRedValue * 255), 
          (int) (kBlueTargetGreenValue * 255), (int) (kBlueTargetBlueValue * 255)));

        updateColorMatcher();
      }
      colorCalibrationEnabled = false;
      ntBlueColorCalibration.setBoolean(false);
    }

    if(ntYellowColorCalibration.getBoolean(false))
    {
      if(colorCalibrationEnabled)
      {
        kYellowTargetRedValue = detectedColor.red;
        kYellowTargetGreenValue = detectedColor.green;
        kYellowTargetBlueValue = detectedColor.blue;

        ntYellowRValue.forceSetDouble(kYellowTargetRedValue);
        ntYellowGValue.forceSetDouble(kYellowTargetGreenValue);
        ntYellowBValue.forceSetDouble(kYellowTargetBlueValue);
        kYellowTarget = new Color(new Color8Bit((int) (kYellowTargetRedValue * 255), 
          (int) (kYellowTargetGreenValue * 255), (int) (kYellowTargetBlueValue * 255)));

        updateColorMatcher();
      }
      colorCalibrationEnabled = false;
      ntYellowColorCalibration.setBoolean(false);
    }
    ntColorCalibrationEnabled.setBoolean(colorCalibrationEnabled);
  }
  
  public boolean colorIsRed()
  {
    return colorIsRed;
  }
  
  public boolean colorIsYellow()
  {
    return colorIsYellow;
  }

  public boolean colorIsBlue()
  {
    return colorIsBlue;
  }

  public boolean colorIsGreen()
  {
    return colorIsGreen;
  }

  public Color getKRedTarget()
  {
    return kRedTarget;
  }

  public Color getKBlueTarget()
  {
    return kBlueTarget;
  }

  public Color getKGreenTarget()
  {
    return kGreenTarget;
  }

  public Color getKYellowTarget()
  {
    return kYellowTarget;
  }

  public Color getColorMatcherColor()
  {
    return colorMatchResult.color;
  }
  public void setNumberOfColorCounts(int numberOfColorCounts)
  {
    ntSTSNumberOfColorCounts.forceSetDouble(numberOfColorCounts);
  }
}
