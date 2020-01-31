/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final Joystick driverJoystick;
  private final Joystick operatorJoystick;

  private final WPI_VictorSPX rotationVictorSPX; 
  private final WPI_VictorSPX leftWinchVictorSPX;
  private final WPI_VictorSPX rightWinchVictorSPX;
  private final WPI_VictorSPX shimmyVictorSPX;
  


  public Arm(WPI_VictorSPX  rotationVictorSPX, WPI_VictorSPX leftWinchVictorSPX, 
      WPI_VictorSPX rightWinchVictorSPX, WPI_VictorSPX shimmyVictorSPX, Joystick driverJoystick, Joystick operatorJoystick) {
    this.rotationVictorSPX = rotationVictorSPX;
    this.leftWinchVictorSPX = leftWinchVictorSPX;
    this.rightWinchVictorSPX = rightWinchVictorSPX;
    this.shimmyVictorSPX = shimmyVictorSPX;
    this.driverJoystick = driverJoystick;
    this.operatorJoystick = operatorJoystick;
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
