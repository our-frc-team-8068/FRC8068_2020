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

  private final WPI_VictorSPX preigniterVictorSPX = new WPI_VictorSPX(20);
  private final WPI_TalonSRX topShooterTalonSRX = new WPI_TalonSRX(21);
  private final WPI_TalonSRX bottomShooterTalonSRX = new WPI_TalonSRX(22);
  
  public Shooter(Joystick driverJoystick) {
    this.driverJoystick = driverJoystick;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
