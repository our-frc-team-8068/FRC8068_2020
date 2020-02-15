/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LogitechGamePad;


public class Magazine extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final Joystick driverJoystick;
  private double setpointDegrees = 0;
  private double scdMagazineSetPointDegrees;
  private boolean scdMagazineUpdatePositionSetpoint = false;

  private final WPI_VictorSPX magazineVictorSPX = new WPI_VictorSPX(40);//50
  private final Encoder magazinePositionEncoder = new Encoder(Constants.DIO_MagazineEncoderBlueSignal, Constants.DIO_MagazineEncoderYellowSignal);
  private final I2C.Port magazineHomePositionColorSensorI2CPort = I2C.Port.kOnboard;
  private final ColorSensorV3 magazineHomePositionColorSensor = new ColorSensorV3(magazineHomePositionColorSensorI2CPort);
  private ShuffleboardTab magazineTab = Shuffleboard.getTab("Magazine"); 
  private boolean hasHomed = false;
  private final DigitalInput colorCalibrationEnabled;

  private NetworkTableEntry ntSTSmagazineSetpointDegrees = 
    magazineTab.add("STSMagazineSetPointDegrees", setpointDegrees).withSize(2, 1).withPosition(0, 0).getEntry();

  private NetworkTableEntry ntSCDmagazineSetpointDegrees = 
    magazineTab.add("SCDMagazineSetPointDegrees", setpointDegrees).withSize(2, 1).withPosition(2, 0).getEntry();

  private NetworkTableEntry ntSCDmagazineUpdatePositionSetpoint = 
    magazineTab.add("SCDMagazineUpdatePositionSetpoint", scdMagazineUpdatePositionSetpoint)
    .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(4, 0).getEntry();

  private double kRedTargetRedValue = 0.0;
  private double kRedTargetGreenValue = 0.0;
  private double kRedTargetBlueValue = 0.0;
  private Color kRedTarget = new Color(new Color8Bit((int) kRedTargetRedValue * 255, 
    (int) kRedTargetGreenValue * 255, (int) kRedTargetBlueValue * 255));

  private double kBlueTargetRedValue = 0.0;
  private double kBlueTargetGreenValue = 0.0;
  private double kBlueTargetBlueValue = 0.0;
  private Color kBlueTarget = new Color(new Color8Bit((int) kBlueTargetRedValue * 255, 
    (int) kBlueTargetGreenValue * 255, (int) kBlueTargetBlueValue * 255));

  private double kGreenTargetRedValue = 0.0;
  private double kGreenTargetGreenValue = 0.0;
  private double kGreenTargetBlueValue = 0.0;
  private Color kGreenTarget = new Color(new Color8Bit((int) kGreenTargetRedValue * 255, 
    (int) kGreenTargetGreenValue * 255, (int) kGreenTargetBlueValue * 255));

  private double kYellowTargetRedValue = 0.0;
  private double kYellowTargetGreenValue = 0.0;
  private double kYellowTargetBlueValue = 0.0;
  private Color kYellowTarget = new Color(new Color8Bit((int) kYellowTargetRedValue * 255, 
    (int) kYellowTargetGreenValue * 255, (int) kYellowTargetBlueValue * 255));

  boolean getRedColorCalibration = false;
  boolean getGreenColorCalibration = false;
  boolean getBlueColorCalibration = false;
  boolean getYellowColorCalibration = false;

  private NetworkTableEntry ntmagazineRedColorCalibration =
    magazineTab.add("magazineRedColorCalibration", getRedColorCalibration)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(0, 1).getEntry();     
  private NetworkTableEntry ntmagazineGreenColorCalibration =
    magazineTab.add("magazineGreenColorCalibration", getGreenColorCalibration)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(2, 1).getEntry();    
  private NetworkTableEntry ntmagazineBlueColorCalibration =
    magazineTab.add("magazineBlueColorCalibration", getBlueColorCalibration)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(4, 1).getEntry();
  private NetworkTableEntry ntmagazineYellowColorCalibration =
    magazineTab.add("magazineYellowColorCalibration", getYellowColorCalibration)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(6, 1).getEntry();

  private NetworkTableEntry ntmagazineRedRValue =
    magazineTab.addPersistent("Red R Value", kRedTargetRedValue).withSize(2, 1).withPosition(0, 2).getEntry();
  private NetworkTableEntry ntmagazineRedGValue =
    magazineTab.addPersistent("Red G Value", kRedTargetGreenValue).withSize(2, 1).withPosition(0, 3).getEntry();
  private NetworkTableEntry ntmagazineRedBValue =
    magazineTab.addPersistent("Red B Value", kRedTargetBlueValue).withSize(2, 1).withPosition(0, 4).getEntry();

  private NetworkTableEntry ntmagazineGreenRValue =
    magazineTab.addPersistent("Green R Value", kGreenTargetRedValue).withSize(2, 1).withPosition(2, 2).getEntry();
  private NetworkTableEntry ntmagazineGreenGValue =
    magazineTab.addPersistent("Green G Value", kGreenTargetGreenValue).withSize(2, 1).withPosition(2, 3).getEntry();
  private NetworkTableEntry ntmagazineGreenBValue =
    magazineTab.addPersistent("Green B Value", kGreenTargetBlueValue).withSize(2, 1).withPosition(2, 4).getEntry();

  private NetworkTableEntry ntmagazineBlueRValue =
    magazineTab.addPersistent("Blue R Value", kBlueTargetRedValue).withSize(2, 1).withPosition(4, 2).getEntry();
  private NetworkTableEntry ntmagazineBlueGValue =
    magazineTab.addPersistent("Blue G Value", kBlueTargetGreenValue).withSize(2, 1).withPosition(4, 3).getEntry();
  private NetworkTableEntry ntmagazineBlueBValue =
    magazineTab.addPersistent("Blue B Value", kBlueTargetBlueValue).withSize(2, 1).withPosition(4, 4).getEntry();

  private NetworkTableEntry ntmagazineYellowRValue =
    magazineTab.addPersistent("Yellow R Value", kYellowTargetRedValue).withSize(2, 1).withPosition(6, 2).getEntry();
  private NetworkTableEntry ntmagazineYellowGValue =
    magazineTab.addPersistent("Yellow G Value", kYellowTargetGreenValue).withSize(2, 1).withPosition(6, 3).getEntry();
  private NetworkTableEntry ntmagazineYellowBValue =
    magazineTab.addPersistent("Yellow B Value", kYellowTargetBlueValue).withSize(2, 1).withPosition(6, 4).getEntry();

  public Magazine(Joystick driverJoystick, DigitalInput colorCalibrationEnabled) {
    this.driverJoystick = driverJoystick;
    this.colorCalibrationEnabled = colorCalibrationEnabled;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ntSTSmagazineSetpointDegrees.setDouble(setpointDegrees);

    if(driverJoystick.getRawButtonPressed(LogitechGamePad.BUTTON_A))
    {
      setSetpointDegrees(180);
    }

    scdMagazineSetPointDegrees = ntSCDmagazineSetpointDegrees.getDouble(0);

    if(ntSCDmagazineUpdatePositionSetpoint.getBoolean(false))
    {
      setSetpointDegrees(scdMagazineSetPointDegrees);
      ntSCDmagazineUpdatePositionSetpoint.setBoolean(false);
    }

    colorCalibration();
  }

  public int getMagazineEncoderValue() {
    return magazinePositionEncoder.getRaw();
  }

  public void setMagazineSpeed(double speed) {
    magazineVictorSPX.set(ControlMode.PercentOutput, speed);
  }

  public int getSetpointEncoderCounts() {
    return convertDegreesToEncoderCounts(setpointDegrees);
  }

  public void setSetpointDegrees(double degrees)
  {
    setpointDegrees = degrees;
  }

  public int convertDegreesToEncoderCounts(double degrees)
  {
    //8192 encoder counts per 360 degrees
    return (int) (degrees * (8192 / 360));
  }

  public double convertEncoderCountsToDegrees(int encoderCounts)
  {
    return encoderCounts * (360 / 8192);
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
    return magazineHomePositionColorSensor.getColor();
  }

  public Color getkRedTarget()
  {
    return kRedTarget;
  }

  public Color getkGreenTarget()
  {
    return kGreenTarget;
  }

  public Color getkBlueTarget()
  {
    return kBlueTarget;
  }

  public Color getkYellowTarget()
  {
    return kYellowTarget;
  }

  public void zeroEncoder()
  {
    magazinePositionEncoder.reset();
  }

  private void colorCalibration()
  {
    boolean isColorCalibrationEnabled = !colorCalibrationEnabled.get();
    Color detectedColor = magazineHomePositionColorSensor.getColor();

    kRedTargetRedValue = ntmagazineRedRValue.getDouble(kRedTargetRedValue);
    kRedTargetGreenValue = ntmagazineRedGValue.getDouble(kRedTargetGreenValue);
    kRedTargetBlueValue = ntmagazineRedBValue.getDouble(kRedTargetBlueValue);

    kGreenTargetRedValue = ntmagazineGreenRValue.getDouble(kGreenTargetRedValue);
    kGreenTargetGreenValue = ntmagazineGreenGValue.getDouble(kGreenTargetGreenValue);
    kGreenTargetBlueValue = ntmagazineGreenBValue.getDouble(kGreenTargetBlueValue);

    kBlueTargetRedValue = ntmagazineBlueRValue.getDouble(kBlueTargetRedValue);
    kBlueTargetGreenValue = ntmagazineBlueGValue.getDouble(kBlueTargetGreenValue);
    kBlueTargetBlueValue = ntmagazineBlueBValue.getDouble(kBlueTargetBlueValue);

    kYellowTargetRedValue = ntmagazineYellowRValue.getDouble(kYellowTargetRedValue);
    kYellowTargetGreenValue = ntmagazineYellowGValue.getDouble(kYellowTargetGreenValue);
    kYellowTargetBlueValue = ntmagazineYellowBValue.getDouble(kYellowTargetBlueValue);

    if(ntmagazineRedColorCalibration.getBoolean(false))
    {
      if(isColorCalibrationEnabled)
      {
        kRedTargetRedValue = detectedColor.red;
        kRedTargetGreenValue = detectedColor.green;
        kRedTargetBlueValue = detectedColor.blue;

        ntmagazineRedRValue.forceSetDouble(kRedTargetRedValue);
        ntmagazineRedGValue.forceSetDouble(kRedTargetGreenValue);
        ntmagazineRedBValue.forceSetDouble(kRedTargetBlueValue);
        kRedTarget = new Color(new Color8Bit((int) kRedTargetRedValue * 255, 
          (int) kRedTargetGreenValue * 255, (int) kRedTargetBlueValue * 255));
      }
      
      ntmagazineRedColorCalibration.setBoolean(false);
    }
    
    if(ntmagazineGreenColorCalibration.getBoolean(false))
    {
      if(isColorCalibrationEnabled)
      {
        kGreenTargetRedValue = detectedColor.red;
        kGreenTargetGreenValue = detectedColor.green;
        kGreenTargetBlueValue = detectedColor.blue; 

        ntmagazineGreenRValue.forceSetDouble(kGreenTargetRedValue);
        ntmagazineGreenGValue.forceSetDouble(kGreenTargetGreenValue);
        ntmagazineGreenBValue.forceSetDouble(kGreenTargetBlueValue);
        kGreenTarget = new Color(new Color8Bit((int) kGreenTargetRedValue * 255, 
          (int) kGreenTargetGreenValue * 255, (int) kGreenTargetBlueValue * 255));
      }

      ntmagazineGreenColorCalibration.setBoolean(false);
    }

    if(ntmagazineBlueColorCalibration.getBoolean(false))
    {
      if(isColorCalibrationEnabled)
      {
        kBlueTargetRedValue = detectedColor.red;
        kBlueTargetGreenValue = detectedColor.green;
        kBlueTargetBlueValue = detectedColor.blue;

        ntmagazineBlueRValue.forceSetDouble(kBlueTargetRedValue);
        ntmagazineBlueGValue.forceSetDouble(kBlueTargetGreenValue);
        ntmagazineBlueBValue.forceSetDouble(kBlueTargetBlueValue);
        kBlueTarget = new Color(new Color8Bit((int) kBlueTargetRedValue * 255, 
          (int) kBlueTargetGreenValue * 255, (int) kBlueTargetBlueValue * 255));
      }

      ntmagazineBlueColorCalibration.setBoolean(false);
    }

    if(ntmagazineYellowColorCalibration.getBoolean(false))
    {
      if(isColorCalibrationEnabled)
      {
        kYellowTargetRedValue = detectedColor.red;
        kYellowTargetGreenValue = detectedColor.green;
        kYellowTargetBlueValue = detectedColor.blue;

        ntmagazineYellowRValue.forceSetDouble(kYellowTargetRedValue);
        ntmagazineYellowGValue.forceSetDouble(kYellowTargetGreenValue);
        ntmagazineYellowBValue.forceSetDouble(kYellowTargetBlueValue);
        kYellowTarget = new Color(new Color8Bit((int) kYellowTargetRedValue * 255, 
          (int) kYellowTargetGreenValue * 255, (int) kYellowTargetBlueValue * 255));
      }

      ntmagazineYellowColorCalibration.setBoolean(false);
    }
  }
}

