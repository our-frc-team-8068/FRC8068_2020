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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogitechGamePad;


public class Magazine extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final Joystick driverJoystick;
  private double setpointDegrees = 0;
  private double scdMagazineSetPointDegrees;
  private boolean scdMagazineUpdatePositionSetpoint = false;

  private final WPI_VictorSPX magazineVictorSPX = new WPI_VictorSPX(50);
  private final Encoder magazinePositionEncoder = new Encoder(0,1);
  private final I2C.Port magazineHomePositionColorSensorI2CPort = I2C.Port.kOnboard;
  private final ColorSensorV3 magazineHomePositionColorSensor = new ColorSensorV3(magazineHomePositionColorSensorI2CPort);
  private ShuffleboardTab magazineTab = Shuffleboard.getTab("Magazine"); 
  private boolean hasHomed = false;

  private NetworkTableEntry ntSTSmagazineSetpointDegrees = 
    magazineTab.add("STSMagazineSetPointDegrees", setpointDegrees).withSize(2, 1).withPosition(0, 0).getEntry();

  private NetworkTableEntry ntSCDmagazineSetpointDegrees = 
    magazineTab.add("SCDMagazineSetPointDegrees", setpointDegrees).withSize(2, 1).withPosition(2, 0).getEntry();

  private NetworkTableEntry ntSCDmagazineUpdatePositionSetpoint = 
    magazineTab.add("SCDMagazineUpdatePositionSetpoint", scdMagazineUpdatePositionSetpoint)
    .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(4, 0).getEntry();

  public Magazine(Joystick driverJoystick) {
    this.driverJoystick = driverJoystick;
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

    //System.out.println("Encoder Value: " + magazinePositionEncoder.getRaw() + " Encoder Setpoint : " + setpointDegrees);
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

  public void zeroEncoder()
  {
    magazinePositionEncoder.reset();
  }
}

