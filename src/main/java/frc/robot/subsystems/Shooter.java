/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final Joystick driverJoystick;

  private final WPI_VictorSPX preigniterVictorSPX;
  private final WPI_TalonSRX topShooterTalonSRX;
  private final WPI_TalonSRX bottomShooterTalonSRX;
  
  public Shooter(WPI_VictorSPX preigniterVictorSPX, WPI_TalonSRX topShooterTalonSRX, WPI_TalonSRX bottomShooterTalonSRX, Joystick driverJoystick) {
    this.preigniterVictorSPX = preigniterVictorSPX;
    this.topShooterTalonSRX = topShooterTalonSRX;
    this.bottomShooterTalonSRX = bottomShooterTalonSRX;
    this.driverJoystick = driverJoystick;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
