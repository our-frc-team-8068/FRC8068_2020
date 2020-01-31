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
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Magazine extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final WPI_VictorSPX magazineVictorSPX;
  private final Encoder magazinePositionEncoder;
  private final ColorSensorV3 magazineHomePositionColorSensor;
  private int setpointEncoderCounts;

  public Magazine(WPI_VictorSPX magazineVictorSPX, Encoder magazinePositionEncoder, ColorSensorV3 magazineHomePositionColorSensor) {
    this.magazineVictorSPX = magazineVictorSPX;
    this.magazinePositionEncoder = magazinePositionEncoder;
    this.magazineHomePositionColorSensor = magazineHomePositionColorSensor;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public int getMagazineEncoderValue() {
    return magazinePositionEncoder.get();
  }

  public void setMagazineSpeed(double speed) {
    magazineVictorSPX.set(ControlMode.PercentOutput, speed);
  }

  public int getSetpointEncoderCounts() {
    return setpointEncoderCounts;
  }
}
