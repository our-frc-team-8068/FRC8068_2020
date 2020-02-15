/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LogitechGamePad;
import frc.robot.commands.CmdMagazineHomeEncoder;


public class Magazine extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final Joystick driverJoystick;
  private double setpointDegrees = 0;
  private double scdPositionSetPointDegrees;
  private double scdProportionalGain;
  private double scdIntegralGain;
  private double scdDerivativeGain;
  private boolean scdUpdatePositionSetpoint = false;

  private final WPI_VictorSPX victorSPX = new WPI_VictorSPX(40);//50
  private final Encoder positionEncoder = new Encoder(Constants.DIO_MagazineEncoderYellowSignal, Constants.DIO_MagazineEncoderBlueSignal);
  private final I2C.Port colorSensorI2CPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(colorSensorI2CPort);
  private ShuffleboardTab magazineControlTab = Shuffleboard.getTab("MagazineControl"); 
  private ShuffleboardTab magazineColorTab = Shuffleboard.getTab("MagazineColor");
  private boolean hasHomed = false;
  private boolean colorIsRed = false;
  private boolean colorIsGreen = false;
  private boolean colorIsBlue = false;
  private boolean colorIsYellow = false;
  private boolean colorCalibrationEnabled = false;

  private ComplexWidget ntHomeEncoder = magazineControlTab.add("ScdHomeMagazine", new CmdMagazineHomeEncoder(this)).withSize(2, 1).withPosition(0, 0);
  private NetworkTableEntry ntHasHomed = magazineControlTab.add("StsMagazineHasHomed", hasHomed).withSize(2, 1).withPosition(2, 0).getEntry();

  private NetworkTableEntry ntStsPositionSetpointDegrees = 
    magazineControlTab.add("StsMagazineSetPointDegrees", setpointDegrees).withSize(2, 1).withPosition(4, 1).getEntry();

  private NetworkTableEntry ntScdPositionSetpointDegrees = 
    magazineControlTab.add("ScdMagazineSetPointDegrees", setpointDegrees).withSize(2, 1).withPosition(0, 1).getEntry();

  private NetworkTableEntry ntScdUpdatePositionSetpoint = 
    magazineControlTab.add("ScdMagazineUpdatePositionSetpoint", scdUpdatePositionSetpoint)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(2, 1).getEntry();

  private NetworkTableEntry ntCurrentPosition =
    magazineControlTab.add("StsMagazineCurrentPosition", 0).withSize(2, 1).withPosition(6, 1).getEntry();
    
  private NetworkTableEntry ntStsProportionalGain = 
    magazineControlTab.addPersistent("StsMagazineProportionalGain ", scdProportionalGain).withSize(2, 1).withPosition(4, 2).getEntry();

  private NetworkTableEntry ntScdProportionalGain = 
    magazineControlTab.addPersistent("ScdMagazineProportionalGain", scdProportionalGain).withSize(2, 1).withPosition(0, 2).getEntry();

  private NetworkTableEntry ntScdUpdateProportionalGain = 
    magazineControlTab.add("ScdMagazineUpdateProportionalGain", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(2, 2).getEntry();

  private NetworkTableEntry ntStsIntegralGain = 
    magazineControlTab.addPersistent("StsMagazineIntegralGain ", scdIntegralGain).withSize(2, 1).withPosition(4, 3).getEntry();
  
  private NetworkTableEntry ntScdIntegralGain = 
    magazineControlTab.addPersistent("ScdMagazineIntegralGain", scdIntegralGain).withSize(2, 1).withPosition(0, 3).getEntry();
  
  private NetworkTableEntry ntScdUpdateIntegralGain = 
    magazineControlTab.add("ScdMagazineUpdateIntegralGain", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(2, 3).getEntry();

  private NetworkTableEntry ntStsDerivativeGain = 
    magazineControlTab.addPersistent("StsMagazineDerivativeGain ", scdDerivativeGain).withSize(2, 1).withPosition(4, 4).getEntry();
    
  private NetworkTableEntry ntScdDerivativeGain = 
    magazineControlTab.addPersistent("ScdMagazineDerivativeGain", scdDerivativeGain).withSize(2, 1).withPosition(0, 4).getEntry();
    
  private NetworkTableEntry ntScdUpdateDerivativeGain = 
    magazineControlTab.add("ScdMagazineUpdateDerivativeGain", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(2, 4).getEntry();


  private ColorMatch colorMatcher = new ColorMatch();

  ColorMatchResult colorMatchResult;

  private double kRedTargetRedValue;
  private double kRedTargetGreenValue;
  private double kRedTargetBlueValue;
  private NetworkTableEntry ntRedRValue =
    magazineColorTab.addPersistent("Magazine Red R Value", 0.0).withSize(2, 1).withPosition(2, 1).getEntry();
  private NetworkTableEntry ntRedGValue =
    magazineColorTab.addPersistent("Magazine Red G Value", 0.0).withSize(2, 1).withPosition(4, 1).getEntry();
  private NetworkTableEntry ntRedBValue =
    magazineColorTab.addPersistent("Magazine Red B Value", 0.0).withSize(2, 1).withPosition(6, 1).getEntry();
  private Color kRedTarget;

  private double kGreenTargetRedValue;
  private double kGreenTargetGreenValue;
  private double kGreenTargetBlueValue;
  private NetworkTableEntry ntGreenRValue =
    magazineColorTab.addPersistent("Magazine Green R Value", kGreenTargetRedValue).withSize(2, 1).withPosition(2, 2).getEntry();
  private NetworkTableEntry ntGreenGValue =
    magazineColorTab.addPersistent("Magazine Green G Value", kGreenTargetGreenValue).withSize(2, 1).withPosition(4, 2).getEntry();
  private NetworkTableEntry ntGreenBValue =
    magazineColorTab.addPersistent("Magazine Green B Value", kGreenTargetBlueValue).withSize(2, 1).withPosition(6, 2).getEntry();
  private Color kGreenTarget;

  private double kBlueTargetRedValue;
  private double kBlueTargetGreenValue;
  private double kBlueTargetBlueValue;
  private NetworkTableEntry ntBlueRValue =
    magazineColorTab.addPersistent("Magazine Blue R Value", kBlueTargetRedValue).withSize(2, 1).withPosition(2, 3).getEntry();
  private NetworkTableEntry ntBlueGValue =
    magazineColorTab.addPersistent("Magazine Blue G Value", kBlueTargetGreenValue).withSize(2, 1).withPosition(4, 3).getEntry();
  private NetworkTableEntry ntBlueBValue =
    magazineColorTab.addPersistent("Magazine Blue B Value", kBlueTargetBlueValue).withSize(2, 1).withPosition(6, 3).getEntry();
  private Color kBlueTarget;

  private double kYellowTargetRedValue;
  private double kYellowTargetGreenValue;
  private double kYellowTargetBlueValue;
  private NetworkTableEntry ntYellowRValue =
    magazineColorTab.addPersistent("Magazine Yellow R Value", kYellowTargetRedValue).withSize(2, 1).withPosition(2, 4).getEntry();
  private NetworkTableEntry ntYellowGValue =
    magazineColorTab.addPersistent("Magazine Yellow G Value", kYellowTargetGreenValue).withSize(2, 1).withPosition(4, 4).getEntry();
  private NetworkTableEntry ntYellowBValue =
    magazineColorTab.addPersistent("Magazine Yellow B Value", kYellowTargetBlueValue).withSize(2, 1).withPosition(6, 4).getEntry();
  private Color kYellowTarget;

  boolean getRedColorCalibration = false;
  boolean getGreenColorCalibration = false;
  boolean getBlueColorCalibration = false;
  boolean getYellowColorCalibration = false;

  private NetworkTableEntry ntRedColorCalibration =
    magazineColorTab.add("Magazine Red Color Calibration", getRedColorCalibration)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(0, 1).getEntry();     
  private NetworkTableEntry ntGreenColorCalibration =
    magazineColorTab.add("Magazine Green Color Calibration", getGreenColorCalibration)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(0, 2).getEntry();    
  private NetworkTableEntry ntBlueColorCalibration =
    magazineColorTab.add("Magazine Blue Color Calibration", getBlueColorCalibration)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(0, 3).getEntry();
  private NetworkTableEntry ntYellowColorCalibration =
    magazineColorTab.add("Magazine Yellow Color Calibration", getYellowColorCalibration)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(0, 4).getEntry();

  private NetworkTableEntry ntColorIsRed = magazineColorTab.add("Is Red", colorIsRed).withSize(1, 1).withPosition(8, 1).getEntry();
  private NetworkTableEntry ntColorIsGreen = magazineColorTab.add("Is Green", colorIsGreen).withSize(1, 1).withPosition(8, 2).getEntry();
  private NetworkTableEntry ntColorIsBlue = magazineColorTab.add("Is Blue", colorIsBlue).withSize(1, 1).withPosition(8, 3).getEntry();
  private NetworkTableEntry ntColorIsYellow = magazineColorTab.add("Is Yellow", colorIsYellow).withSize(1, 1).withPosition(8, 4).getEntry();

  private NetworkTableEntry ntColorCalibrationEnabled = magazineColorTab.add("Magazine Color Calibration Enabled", colorCalibrationEnabled)
    .withWidget(BuiltInWidgets.kToggleSwitch).withSize(2, 1).withPosition(6, 0).getEntry();

  public Magazine(Joystick driverJoystick) {
    this.driverJoystick = driverJoystick;

    getNewShuffleboardData();

    kRedTarget = new Color(new Color8Bit((int) (kRedTargetRedValue * 255), (int) (kRedTargetGreenValue * 255), (int) (kRedTargetBlueValue * 255)));
    kGreenTarget  = new Color(new Color8Bit((int) (kGreenTargetRedValue * 255), (int) (kGreenTargetGreenValue * 255), (int) (kGreenTargetBlueValue * 255)));
    kBlueTarget = new Color(new Color8Bit((int) (kBlueTargetRedValue * 255), (int) (kBlueTargetGreenValue * 255), (int) (kBlueTargetBlueValue * 255)));
    kYellowTarget  = new Color(new Color8Bit((int) (kYellowTargetRedValue * 255), (int) (kYellowTargetGreenValue * 255), (int) (kYellowTargetBlueValue * 255)));

    updateColorMatcher();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ntStsPositionSetpointDegrees.setDouble(setpointDegrees);
    ntStsProportionalGain.setDouble(scdProportionalGain);
    ntStsIntegralGain.setDouble(scdIntegralGain);
    ntStsDerivativeGain.setDouble(scdDerivativeGain);

    colorCalibrationEnabled = ntColorCalibrationEnabled.getBoolean(false);

    ntCurrentPosition.forceSetDouble(getPositionInDegrees());
    //System.out.println("Current Position: " + positionEncoder.getRaw());

    if(driverJoystick.getRawButtonPressed(LogitechGamePad.BUTTON_A))
    {
      setSetpointDegrees(180);
    }

    scdPositionSetPointDegrees = ntScdPositionSetpointDegrees.getDouble(0);

    if(ntScdUpdatePositionSetpoint.getBoolean(false))
    {
      setSetpointDegrees(scdPositionSetPointDegrees);
      ntScdUpdatePositionSetpoint.setBoolean(false);
    }

    if(ntScdUpdateProportionalGain.getBoolean(true))
    {
      scdProportionalGain = ntScdProportionalGain.getDouble(0);
      ntScdUpdateProportionalGain.setBoolean(false);
    }
    
    if(ntScdUpdateIntegralGain.getBoolean(true))
    {
      scdIntegralGain = ntScdIntegralGain.getDouble(0);
      ntScdUpdateIntegralGain.setBoolean(false);
    }

    if(ntScdUpdateDerivativeGain.getBoolean(true))
    {
      scdDerivativeGain = ntScdDerivativeGain.getDouble(0);
      ntScdUpdateDerivativeGain.setBoolean(false);
    }

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

    ntHasHomed.setBoolean(hasHomed);

    colorCalibration();
  }

  /*public int getEncoderValue() {
    return positionEncoder.getRaw();
  }*/

  public void setSpeed(double speed) {
    victorSPX.set(ControlMode.PercentOutput, speed);
  }

  /*public int getSetpointEncoderCounts() {
    return convertDegreesToEncoderCounts(setpointDegrees);
  }*/

  public double getSetpointInDegrees()
  {
    return setpointDegrees;
  }

  public void setSetpointDegrees(double degrees)
  {
    setpointDegrees = degrees;
  }

  /*public int convertDegreesToEncoderCounts(double degrees)
  {
    //8192 encoder counts per 360 degrees
    return (int) (degrees * (8192 / 360.0));
  }*/

  public double getPositionInDegrees()
  {
    return (positionEncoder.getRaw() % 8192) * (360.0 / 8192);
  }

  public boolean getHasHomed() 
  {
    return hasHomed;
  }

  public void setHasHomed(boolean hasHomed)
  {
    this.hasHomed = hasHomed;
  }

  public Color getColorSensorColor()
  {
    return colorSensor.getColor();
  }

  public boolean colorIsRed()
  {
    return colorIsRed;
  }

  public boolean colorIsGreen()
  {
    return colorIsGreen;
  }
  
  public boolean colorIsBlue()
  {
    return colorIsBlue;
  }
  
  public boolean colorIsYellow()
  {
    return colorIsYellow;
  }

  public void zeroEncoder()
  {
    positionEncoder.reset();
  }

  private void updateColorMatcher()
  {
    colorMatcher = new ColorMatch();
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kYellowTarget);
  }

  private void getNewShuffleboardData()
  {
    kRedTargetRedValue = ntRedRValue.getDouble(kRedTargetRedValue);
    kRedTargetGreenValue = ntRedGValue.getDouble(kRedTargetGreenValue);
    kRedTargetBlueValue = ntRedBValue.getDouble(kRedTargetBlueValue);

    kGreenTargetRedValue = ntGreenRValue.getDouble(kGreenTargetRedValue);
    kGreenTargetGreenValue = ntGreenGValue.getDouble(kGreenTargetGreenValue);
    kGreenTargetBlueValue = ntGreenBValue.getDouble(kGreenTargetBlueValue);

    kBlueTargetRedValue = ntBlueRValue.getDouble(kBlueTargetRedValue);
    kBlueTargetGreenValue = ntBlueGValue.getDouble(kBlueTargetGreenValue);
    kBlueTargetBlueValue = ntBlueBValue.getDouble(kBlueTargetBlueValue);

    kYellowTargetRedValue = ntYellowRValue.getDouble(kYellowTargetRedValue);
    kYellowTargetGreenValue = ntYellowGValue.getDouble(kYellowTargetGreenValue);
    kYellowTargetBlueValue = ntYellowBValue.getDouble(kYellowTargetBlueValue);

    scdProportionalGain = ntStsProportionalGain.getDouble(scdProportionalGain);
    scdIntegralGain = ntStsIntegralGain.getDouble(scdIntegralGain);
    scdDerivativeGain = ntStsDerivativeGain.getDouble(scdDerivativeGain);
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

  public double getProportionalGain()
  {
    return scdProportionalGain;
  }

  public double getIntegralGain()
  {
    return scdIntegralGain;
  }

  public double getDerivativeGain()
  {
    return scdDerivativeGain;
  }
}

