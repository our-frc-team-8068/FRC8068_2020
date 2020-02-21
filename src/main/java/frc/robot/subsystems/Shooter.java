/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final Joystick driverJoystick;

  private final WPI_VictorSPX preigniterVictorSPX = new WPI_VictorSPX(20);
  private final WPI_TalonSRX topShooterTalonSRX = new WPI_TalonSRX(21);
  private final WPI_TalonSRX bottomShooterTalonSRX = new WPI_TalonSRX(22);

  private ShuffleboardTab shooterControlTab = Shuffleboard.getTab("ShooterControl"); 
  
  private NetworkTableEntry ntScdProportionalGain = 
    shooterControlTab.add("ScdShooterProportionalGain", 69.0).withSize(2, 1).withPosition(0, 0).getEntry();
    
  private NetworkTableEntry ntScdUpdateProportionalGain = 
    shooterControlTab.add("ScdShooterUpdateProportionalGain", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1).withPosition(2, 0).getEntry();
  
  private NetworkTableEntry ntStsProportionalGain = 
    shooterControlTab.addPersistent("StsShooterProportionalGain ", 69.0).withSize(2, 1).withPosition(4, 0).getEntry();

    private NetworkTableEntry ntScdIntegralGain = 
    shooterControlTab.add("ScdShooterIntegralGain", 69.0).withSize(2, 1).withPosition(0, 1).getEntry();
    
  private NetworkTableEntry ntScdUpdateIntegralGain = 
    shooterControlTab.add("ScdShooterUpdateIntegralGain", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1).withPosition(2, 1).getEntry();
  
  private NetworkTableEntry ntStsIntegralGain = 
    shooterControlTab.addPersistent("StsShooterIntegralGain ", 69.0).withSize(2, 1).withPosition(4, 1).getEntry();  

  private NetworkTableEntry ntScdDerivativeGain = 
    shooterControlTab.add("ScdShooterDerivativeGain", 69.0).withSize(2, 1).withPosition(0, 2).getEntry();
    
  private NetworkTableEntry ntScdUpdateDerivativeGain = 
    shooterControlTab.add("ScdShooterUpdateDerivativeGain", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1).withPosition(2, 2).getEntry();
  
  private NetworkTableEntry ntStsDerivativeGain = 
    shooterControlTab.addPersistent("StsShooterDerivativeGain ", 69.0).withSize(2, 1).withPosition(4, 2).getEntry();

  
  public Shooter(Joystick driverJoystick) {
    this.driverJoystick = driverJoystick;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
