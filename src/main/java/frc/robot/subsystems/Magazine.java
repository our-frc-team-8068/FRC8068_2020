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

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogitechGamePad;


public class Magazine extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final Joystick driverJoystick;
  private int setpointEncoderCounts;

  private final WPI_VictorSPX magazineVictorSPX = new WPI_VictorSPX(50);
  private final Encoder magazinePositionEncoder = new Encoder(0,1);
  private final I2C.Port magazineHomePositionColorSensorI2CPort = I2C.Port.kMXP;
  private final ColorSensorV3 magazineHomePositionColorSensor = new ColorSensorV3(magazineHomePositionColorSensorI2CPort);

  public Magazine(Joystick driverJoystick) {
    this.driverJoystick = driverJoystick;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(driverJoystick.getRawButtonPressed(LogitechGamePad.BUTTON_A))
    {
      setpointEncoderCounts += 8192 / 5;
    }
    System.out.println("Encoder Value: " + magazinePositionEncoder.getRaw() + " Encoder Setpoint : " + setpointEncoderCounts);
  }

  public int getMagazineEncoderValue() {
    return magazinePositionEncoder.getRaw();
  }

  public void setMagazineSpeed(double speed) {
    magazineVictorSPX.set(ControlMode.PercentOutput, speed);
  }

  public int getSetpointEncoderCounts() {
    return setpointEncoderCounts;
  }
}
