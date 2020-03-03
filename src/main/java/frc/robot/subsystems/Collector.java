/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Collector extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private final Joystick driverJoystick;
  private final Joystick operatorJoystick;
  
  private final WPI_VictorSPX collectorVictorSPX = new WPI_VictorSPX(45);
  private final DoubleSolenoid collectorDeploymentSolenoid = new DoubleSolenoid(Constants.DIO_CollectorCylinderExtend,
    Constants.DIO_CollectorCYlinderRetract);

  private double collectorSpeed;

  private ShuffleboardTab collectorControlTab = Shuffleboard.getTab("CollectorControl"); 

  private NetworkTableEntry ntStsSpeed = 
    collectorControlTab.addPersistent("StsCollectorSpeed", 69.0).withSize(2, 1).withPosition(4, 0).getEntry();

  private NetworkTableEntry ntScdSpeed = 
    collectorControlTab.add("ScdCollectorSpeed", 69.0).withSize(2, 1).withPosition(0, 0).getEntry();

  private NetworkTableEntry ntScdUpdateSpeed = 
    collectorControlTab.add("ScdCollectorUpdateCollectorSpeed", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(2, 0).getEntry();


  public Collector(Joystick driverJoystick, Joystick operatorJoystick) {
    this.driverJoystick = driverJoystick;
    this.operatorJoystick = operatorJoystick;
    collectorSpeed = ntStsSpeed.getDouble(69.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(ntScdUpdateSpeed.getBoolean(false))
    {
      collectorSpeed = ntScdSpeed.getDouble(69.0);
      ntStsSpeed.forceSetDouble(collectorSpeed);
      ntScdUpdateSpeed.setBoolean(false);
    }

  }

  public void collect(double speed)
  {
    collectorVictorSPX.set(speed);
  }

  public void extendCollectorCylinder()
  {
    collectorDeploymentSolenoid.set(Value.kForward);

  }
  
  public void retractCollectorCylinder()
  {
    collectorDeploymentSolenoid.set(Value.kReverse);

  }
}
