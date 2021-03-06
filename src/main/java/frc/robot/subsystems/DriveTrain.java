/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogitechGamePad;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.Joystick;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final WPI_TalonSRX leftDriveTalonSRX = new WPI_TalonSRX(10);
  private final WPI_TalonSRX rightDriveTalonSRX = new WPI_TalonSRX(11);
  private final WPI_VictorSPX leftDriveVictorSPX = new WPI_VictorSPX(12);
  private final WPI_VictorSPX rightDriveVictorSPX = new WPI_VictorSPX(13);

  private final Joystick driverJoystick;

  private boolean driveTrainInvertDirection = false;
  
  public DriveTrain(Joystick driverJoystick ) {
  this.driverJoystick = driverJoystick;

    rightDriveTalonSRX.configFactoryDefault();
    rightDriveTalonSRX.set(ControlMode.PercentOutput, 0);
    rightDriveTalonSRX.setInverted(true);
    rightDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
    rightDriveTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    
    //rightDriveVictorSPX.set(ControlMode.Follower, rightDriveTalonSRX.getDeviceID());
    rightDriveVictorSPX.configFactoryDefault();
    rightDriveVictorSPX.setNeutralMode(NeutralMode.Brake);
    rightDriveVictorSPX.setInverted(true);

    leftDriveTalonSRX.configFactoryDefault();
    leftDriveTalonSRX.set(ControlMode.PercentOutput, 0);
    leftDriveTalonSRX.setInverted(false);
    leftDriveTalonSRX.setNeutralMode(NeutralMode.Brake);
    leftDriveTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    
    //leftDriveVictorSPX.set(ControlMode.Follower, leftDriveTalonSRX.getDeviceID());
    leftDriveVictorSPX.configFactoryDefault();
    leftDriveVictorSPX.setNeutralMode(NeutralMode.Brake);
    leftDriveVictorSPX.setInverted(false);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  
  public void setDriveTrainSpeeds(double leftSideSpeed, double rightSideSpeed) {
    leftDriveTalonSRX.set(leftSideSpeed);
    rightDriveTalonSRX.set(rightSideSpeed);
    leftDriveVictorSPX.set(leftSideSpeed);
    rightDriveVictorSPX.set(rightSideSpeed);
  
  }

  public void driveTrainInvertDirection(){
    driveTrainInvertDirection = !driveTrainInvertDirection;
  }

  public boolean getDriveTrainInvertDirection(){
    return driveTrainInvertDirection;
  }
}
