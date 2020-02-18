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
  private double stsProportionalGain;
  private double stsIntegralGain;
  private double stsDerivativeGain;
  private boolean scdUpdatePositionSetpoint = false;

  private final WPI_VictorSPX victorSPX = new WPI_VictorSPX(50);//50
  private final Encoder positionEncoder = new Encoder(Constants.DIO_MagazineEncoderYellowSignal, Constants.DIO_MagazineEncoderBlueSignal);
  private final I2C.Port colorSensorI2CPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(colorSensorI2CPort);
  private ShuffleboardTab magazineControlTab = Shuffleboard.getTab("MagazineControl"); 
  private ShuffleboardTab magazineColorTab = Shuffleboard.getTab("MagazineColor");
  private boolean hasHomed = false;
  private boolean colorIsRed = false;
  private boolean colorIsYellow = false;
  private boolean colorIsLexan = false;
  private boolean colorCalibrationEnabled = false;

  private ComplexWidget ntHomeEncoder = magazineControlTab.add("ScdHomeMagazine", new CmdMagazineHomeEncoder(this)).withSize(2, 1).withPosition(0, 0);
  private NetworkTableEntry ntHasHomed = magazineControlTab.add("StsMagazineHasHomed", hasHomed).withSize(2, 1).withPosition(2, 0).getEntry();

  private NetworkTableEntry ntStsPositionSetpointDegrees = 
    magazineControlTab.add("StsMagazineSetPointDegrees", 69.0).withSize(2, 1).withPosition(4, 1).getEntry();

  private NetworkTableEntry ntScdPositionSetpointDegrees = 
    magazineControlTab.add("ScdMagazineSetPointDegrees", 69.0).withSize(2, 1).withPosition(0, 1).getEntry();

  private NetworkTableEntry ntScdUpdatePositionSetpoint = 
    magazineControlTab.add("ScdMagazineUpdatePositionSetpoint", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(2, 1).getEntry();

  private NetworkTableEntry ntCurrentPosition =
    magazineControlTab.add("StsMagazineCurrentPosition", 69.0).withSize(2, 1).withPosition(6, 1).getEntry();
    
  private NetworkTableEntry ntStsProportionalGain = 
    magazineControlTab.addPersistent("StsMagazineProportionalGain ", 69.0).withSize(2, 1).withPosition(4, 2).getEntry();

  private NetworkTableEntry ntScdProportionalGain = 
    magazineControlTab.add("ScdMagazineProportionalGain", 69.0).withSize(2, 1).withPosition(0, 2).getEntry();

  private NetworkTableEntry ntScdUpdateProportionalGain = 
    magazineControlTab.add("ScdMagazineUpdateProportionalGain", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(2, 2).getEntry();

  private NetworkTableEntry ntStsIntegralGain = 
    magazineControlTab.addPersistent("StsMagazineIntegralGain ", 69.0).withSize(2, 1).withPosition(4, 3).getEntry();
  
  private NetworkTableEntry ntScdIntegralGain = 
    magazineControlTab.add("ScdMagazineIntegralGain", 69.0).withSize(2, 1).withPosition(0, 3).getEntry();
  
  private NetworkTableEntry ntScdUpdateIntegralGain = 
    magazineControlTab.add("ScdMagazineUpdateIntegralGain", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(2, 3).getEntry();

  private NetworkTableEntry ntStsDerivativeGain = 
    magazineControlTab.addPersistent("StsMagazineDerivativeGain ", 69.0).withSize(2, 1).withPosition(4, 4).getEntry();
    
  private NetworkTableEntry ntScdDerivativeGain = 
    magazineControlTab.add("ScdMagazineDerivativeGain", 69.0).withSize(2, 1).withPosition(0, 4).getEntry();
    
  private NetworkTableEntry ntScdUpdateDerivativeGain = 
    magazineControlTab.add("ScdMagazineUpdateDerivativeGain", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(2, 4).getEntry();


  private ColorMatch colorMatcher = new ColorMatch();

  ColorMatchResult colorMatchResult;

  private double kRedTargetRedValue;
  private double kRedTargetGreenValue;
  private double kRedTargetBlueValue;
  private NetworkTableEntry ntRedRValue =
    magazineColorTab.addPersistent("Magazine Red R Value", 69.0).withSize(2, 1).withPosition(2, 1).getEntry();
  private NetworkTableEntry ntRedGValue =
    magazineColorTab.addPersistent("Magazine Red G Value", 69.0).withSize(2, 1).withPosition(4, 1).getEntry();
  private NetworkTableEntry ntRedBValue =
    magazineColorTab.addPersistent("Magazine Red B Value", 69.0).withSize(2, 1).withPosition(6, 1).getEntry();
  private Color kRedTarget;

  private double kYellowTargetRedValue;
  private double kYellowTargetGreenValue;
  private double kYellowTargetBlueValue;
  private NetworkTableEntry ntYellowRValue =
    magazineColorTab.addPersistent("Magazine Yellow R Value", 69.0).withSize(2, 1).withPosition(2, 2).getEntry();
  private NetworkTableEntry ntYellowGValue =
    magazineColorTab.addPersistent("Magazine Yellow G Value", 69.0).withSize(2, 1).withPosition(4, 2).getEntry();
  private NetworkTableEntry ntYellowBValue =
    magazineColorTab.addPersistent("Magazine Yellow B Value", 69.0).withSize(2, 1).withPosition(6, 2).getEntry();
  private Color kYellowTarget;

  private double kLexanTargetRedValue;
  private double kLexanTargetGreenValue;
  private double kLexanTargetBlueValue;
  private NetworkTableEntry ntLexanRValue =
    magazineColorTab.addPersistent("Magazine Lexan R Value", 69.0).withSize(2, 1).withPosition(2, 3).getEntry();
  private NetworkTableEntry ntLexanGValue =
    magazineColorTab.addPersistent("Magazine Lexan G Value", 69.0).withSize(2, 1).withPosition(4, 3).getEntry();
  private NetworkTableEntry ntLexanBValue =
    magazineColorTab.addPersistent("Magazine Lexan B Value", 69.0).withSize(2, 1).withPosition(6, 3).getEntry();
  private Color kLexanTarget;

  boolean getRedColorCalibration = false;
  boolean getYellowColorCalibration = false;
  boolean getLexanColorCalibration = false;

  private NetworkTableEntry ntRedColorCalibration =
    magazineColorTab.add("Magazine Red Color Calibration", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(0, 1).getEntry();   
  private NetworkTableEntry ntYellowColorCalibration = 
    magazineColorTab.add("Magazine Yellow Color Calibration", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(0, 2).getEntry();
  private NetworkTableEntry ntLexanColorCalibration = 
    magazineColorTab.add("Magazine Lexan Color Calibration", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(0, 3).getEntry();

  private NetworkTableEntry ntColorIsRed = magazineColorTab.add("Is Red", colorIsRed).withSize(1, 1).withPosition(8, 1).getEntry();
  private NetworkTableEntry ntColorIsYellow = magazineColorTab.add("Is Yellow", colorIsYellow).withSize(1, 1).withPosition(8, 2).getEntry();
  private NetworkTableEntry ntColorIsLexan = magazineColorTab.add("Is Lexan", colorIsLexan).withSize(1, 1).withPosition(8, 3).getEntry();


  private NetworkTableEntry ntColorCalibrationEnabled = magazineColorTab.add("Magazine Color Calibration Enabled", colorCalibrationEnabled)
    .withWidget(BuiltInWidgets.kToggleSwitch).withSize(2, 1).withPosition(0, 0).getEntry();

  private NetworkTableEntry ntMagazineColorSensorCurrentRed = magazineColorTab.add("MagazineColorSensorCurrentRed", 69.0).withSize(2,1).withPosition(2,0).getEntry();
  private NetworkTableEntry ntMagazineColorSensorCurrentGreen = magazineColorTab.add("MagazineColorSensorCurentGreen", 69.0).withSize(2, 1).withPosition(4, 0).getEntry();
  private NetworkTableEntry ntMagazineColorSensorCurrentBlue = magazineColorTab.add("MagazineColorSensorCurrentBlue", 69.0).withSize(2, 1).withPosition(6, 0).getEntry();
    
  public Magazine(Joystick driverJoystick) {
    this.driverJoystick = driverJoystick;

    getNewShuffleboardData();

    kRedTarget = new Color(new Color8Bit((int) (kRedTargetRedValue * 255), (int) (kRedTargetGreenValue * 255), (int) (kRedTargetBlueValue * 255)));
    kYellowTarget  = new Color(new Color8Bit((int) (kYellowTargetRedValue * 255), (int) (kYellowTargetGreenValue * 255), (int) (kYellowTargetBlueValue * 255)));
    kLexanTarget = new Color(new Color8Bit((int) (kLexanTargetRedValue * 255), (int) (kLexanTargetGreenValue * 255), (int) (kLexanTargetBlueValue * 255)));

    updateColorMatcher();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ntMagazineColorSensorCurrentRed.forceSetDouble(colorSensor.getColor().red);
    ntMagazineColorSensorCurrentGreen.forceSetDouble(colorSensor.getColor().green);
    ntMagazineColorSensorCurrentBlue.forceSetDouble(colorSensor.getColor().blue);

    ntStsPositionSetpointDegrees.setDouble(setpointDegrees);
    
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

    colorMatchResult = colorMatcher.matchClosestColor(colorSensor.getColor()); 

    if(colorMatchResult.color == kRedTarget)
    {
      colorIsRed = true;
      colorIsYellow = false;
      colorIsLexan = false;
    }
    else if(colorMatchResult.color == kYellowTarget)
    {
      colorIsRed = false;
      colorIsYellow = true;
      colorIsLexan = false;
    }
    else if(colorMatchResult.color == kLexanTarget)
    {
      colorIsRed = false;
      colorIsYellow = false;
      colorIsLexan = true;
    }

    ntColorIsRed.setBoolean(colorIsRed);
    ntColorIsYellow.setBoolean(colorIsYellow);
    ntColorIsLexan.setBoolean(colorIsLexan);

    ntHasHomed.setBoolean(hasHomed);

    colorCalibration();
    PIDCalibrations();
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
    colorMatcher.addColorMatch(kYellowTarget);
    colorMatcher.addColorMatch(kLexanTarget);
  }

  private void getNewShuffleboardData()
  {
    kRedTargetRedValue = ntRedRValue.getDouble(69.0);
    kRedTargetGreenValue = ntRedGValue.getDouble(69.0);
    kRedTargetBlueValue = ntRedBValue.getDouble(69.0);

    kYellowTargetRedValue = ntYellowRValue.getDouble(69.0);
    kYellowTargetGreenValue = ntYellowGValue.getDouble(69.0);
    kYellowTargetBlueValue = ntYellowBValue.getDouble(69.0);

    kLexanTargetRedValue = ntLexanRValue.getDouble(69.0);
    kLexanTargetGreenValue = ntLexanGValue.getDouble(69.0);
    kLexanTargetBlueValue = ntLexanBValue.getDouble(69.0);

    stsProportionalGain = ntStsProportionalGain.getDouble(69.0);
    stsIntegralGain = ntStsIntegralGain.getDouble(69.0);
    stsDerivativeGain = ntStsDerivativeGain.getDouble(69.0);
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

    if(ntLexanColorCalibration.getBoolean(false))
    {
      if(colorCalibrationEnabled)
      {
        kLexanTargetRedValue = detectedColor.red;
        kLexanTargetGreenValue = detectedColor.green;
        kLexanTargetBlueValue = detectedColor.blue;

        ntLexanRValue.forceSetDouble(kLexanTargetRedValue);
        ntLexanGValue.forceSetDouble(kLexanTargetGreenValue);
        ntLexanBValue.forceSetDouble(kLexanTargetBlueValue);
        kLexanTarget = new Color(new Color8Bit((int) (kLexanTargetRedValue * 255), 
          (int) (kLexanTargetGreenValue * 255), (int) (kLexanTargetBlueValue * 255)));

        updateColorMatcher();
      }
      colorCalibrationEnabled = false;
      ntLexanColorCalibration.setBoolean(false);
    }
    ntColorCalibrationEnabled.setBoolean(colorCalibrationEnabled);
  }

  public double getProportionalGain()
  {
    return stsProportionalGain;
  }

  public double getIntegralGain()
  {
    return stsIntegralGain;
  }

  public double getDerivativeGain()
  {
    return stsDerivativeGain;
  }

  public void PIDCalibrations()
  {
    if(ntScdUpdateProportionalGain.getBoolean(false))
    {
      stsProportionalGain = ntScdProportionalGain.getDouble(69.0);
      ntStsProportionalGain.forceSetDouble(stsProportionalGain);
      ntScdUpdateProportionalGain.setBoolean(false);
    }
    
    if(ntScdUpdateIntegralGain.getBoolean(false))
    {
      stsIntegralGain = ntScdIntegralGain.getDouble(69.0);
      ntStsIntegralGain.forceSetDouble(stsIntegralGain);
      ntScdUpdateIntegralGain.setBoolean(false);
    }

    if(ntScdUpdateDerivativeGain.getBoolean(false))
    {
      stsDerivativeGain = ntScdDerivativeGain.getDouble(69.0);
      ntStsDerivativeGain.forceSetDouble(stsDerivativeGain);
      ntScdUpdateDerivativeGain.setBoolean(false);
    }

  }
}

