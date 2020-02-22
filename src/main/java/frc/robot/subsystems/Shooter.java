/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.CmdShooterShoot;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final Joystick driverJoystick;

  private final WPI_VictorSPX preigniterVictorSPX = new WPI_VictorSPX(20);
  public final WPI_TalonSRX topShooterTalonSRX = new WPI_TalonSRX(21); // 21
  public final WPI_TalonSRX bottomShooterTalonSRX = new WPI_TalonSRX(22);
  //public final Compressor pneumaticsCompressor = new Compressor(70);
  //public final Solenoid preigniterSolenoid = new Solenoid(59);
  private double stsProportionalGain;
  private double stsIntegralGain;
  private double stsDerivativeGain;
  public double stsShooterUpperShooterHighSpeed;
  public double stsShooterLowerShooterHighSpeed;
  public double stsShooterUpperShooterLowSpeed;
  public double stsShooterLowerShooterLowSpeed;
  private boolean shooterHighSpeed = false;
  private boolean shooterLowSpeed = false;
  //private boolean cmdEnableShooterShoot = false;

  private ShuffleboardTab shooterControlTab = Shuffleboard.getTab("ShooterControl"); 
  
  private NetworkTableEntry ntScdProportionalGain = 
    shooterControlTab.add("ScdShooterProportionalGain", 69.0).withSize(2, 1).withPosition(0, 0).getEntry();
    
  private NetworkTableEntry ntScdUpdateProportionalGain = 
    shooterControlTab.add("ScdShooterUpdateProportionalGain", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1).withPosition(2, 0).getEntry();
  
  private NetworkTableEntry ntStsProportionalGain = 
    shooterControlTab.addPersistent("StsShooterProportionalGain ", 69.0).withSize(2, 1).withPosition(3, 0).getEntry();

    private NetworkTableEntry ntScdIntegralGain = 
    shooterControlTab.add("ScdShooterIntegralGain", 69.0).withSize(2, 1).withPosition(0, 1).getEntry();
    
  private NetworkTableEntry ntScdUpdateIntegralGain = 
    shooterControlTab.add("ScdShooterUpdateIntegralGain", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1).withPosition(2, 1).getEntry();
  
  private NetworkTableEntry ntStsIntegralGain = 
    shooterControlTab.addPersistent("StsShooterIntegralGain ", 69.0).withSize(2, 1).withPosition(3, 1).getEntry();  

  private NetworkTableEntry ntScdDerivativeGain = 
    shooterControlTab.add("ScdShooterDerivativeGain", 69.0).withSize(2, 1).withPosition(0, 2).getEntry();
    
  private NetworkTableEntry ntScdUpdateDerivativeGain = 
    shooterControlTab.add("ScdShooterUpdateDerivativeGain", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1).withPosition(2, 2).getEntry();
  
  private NetworkTableEntry ntStsDerivativeGain = 
    shooterControlTab.addPersistent("StsShooterDerivativeGain ", 69.0).withSize(2, 1).withPosition(3, 2).getEntry();

  private NetworkTableEntry ntScdUpperShooterHighSpeed = 
    shooterControlTab.add("ScdShooterUpperShooterHighSpeed", 69.0).withSize(2, 1).withPosition(6, 0).getEntry();
    
  private NetworkTableEntry ntScdUpdateUpperShooterHighSpeed = 
    shooterControlTab.add("ScdShooterUpdateUpperShooterHighSpeed", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1).withPosition(8, 0).getEntry();
  
  private NetworkTableEntry ntStsUpperShooterHighSpeed = 
    shooterControlTab.addPersistent("StsShooterUpperShooterHighSpeed", 69.0).withSize(2, 1).withPosition(9, 0).getEntry();      
  
    private NetworkTableEntry ntScdLowerShooterHighSpeed = 
    shooterControlTab.add("ScdShooterLowerShooterHighSpeed", 69.0).withSize(2, 1).withPosition(6, 1).getEntry();
    
  private NetworkTableEntry ntScdUpdateLowerShooterHighSpeed = 
    shooterControlTab.add("ScdShooterUpdateLowerShooterHighSpeed", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1).withPosition(8, 1).getEntry();
  
  private NetworkTableEntry ntStsLowerShooterHighSpeed = 
    shooterControlTab.addPersistent("StsShooterLowerShooterHighSpeed", 69.0).withSize(2, 1).withPosition(9, 1).getEntry(); 
  
  private NetworkTableEntry ntScdUpperShooterLowSpeed = 
    shooterControlTab.add("ScdShooterUpperShooterLowSpeed", 69.0).withSize(2, 1).withPosition(6, 2).getEntry();
    
  private NetworkTableEntry ntScdUpdateUpperShooterLowSpeed = 
    shooterControlTab.add("ScdShooterUpdateUpperShooterLowSpeed", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1).withPosition(8, 2).getEntry();
  
  private NetworkTableEntry ntStsUpperShooterLowSpeed = 
    shooterControlTab.addPersistent("StsShooterUpperShooterLowSpeed", 69.0).withSize(2, 1).withPosition(9, 2).getEntry();
    
  private NetworkTableEntry ntScdLowerShooterLowSpeed = 
    shooterControlTab.add("ScdShooterLowerShooterLowSpeed", 69.0).withSize(2, 1).withPosition(6, 3).getEntry();
    
  private NetworkTableEntry ntScdUpdateLowerShooterLowSpeed = 
    shooterControlTab.add("ScdShooterUpdateLowerShooterLowSpeed", false)
      .withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1).withPosition(8, 3).getEntry();
  
  private NetworkTableEntry ntStsLowerShooterLowSpeed = 
    shooterControlTab.addPersistent("StsShooterLowerShooterLowSpeed", 69.0).withSize(2, 1).withPosition(9, 3).getEntry();
  
  private NetworkTableEntry ntUpperTalonCurrentVelocity = 
    shooterControlTab.addPersistent("ShooterUpperTalonCurrentVelocity", 69.0).withSize(2, 1).withPosition(6, 4).getEntry();  
    
  private NetworkTableEntry ntLowerTalonCurrentVelocity = 
    shooterControlTab.addPersistent("ShooterLowerTalonCurrentVelocity", 69.0).withSize(2, 1).withPosition(8, 4).getEntry();
  
    private NetworkTableEntry ntEnableCmdShooterShoot = 
    shooterControlTab.add("enableCmdShooterShoot", false)
      .withWidget(BuiltInWidgets.kToggleSwitch).withSize(1, 1).withPosition(9, 4).getEntry();
    
  public Shooter(Joystick driverJoystick) {
    this.driverJoystick = driverJoystick;

    getNewShuffleboardData();
    topShooterTalonSRX.config_kP(0, stsProportionalGain);
    bottomShooterTalonSRX.config_kP(0, stsProportionalGain);
    topShooterTalonSRX.config_kI(0, stsIntegralGain);
    bottomShooterTalonSRX.config_kI(0, stsIntegralGain);
    topShooterTalonSRX.config_kD(0, stsDerivativeGain);
    bottomShooterTalonSRX.config_kD(0, stsDerivativeGain);
    
    topShooterTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    bottomShooterTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    topShooterTalonSRX.setInverted(true);
    bottomShooterTalonSRX.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    ntUpperTalonCurrentVelocity.forceSetDouble(topShooterTalonSRX.getSelectedSensorVelocity());
    ntLowerTalonCurrentVelocity.forceSetDouble(bottomShooterTalonSRX.getSelectedSensorVelocity());
    PIDCalibrations();

    //cmdEnableShooterShoot = ntEnableCmdShooterShoot.getBoolean(false);

  }

  public void getNewShuffleboardData()
  {
    stsProportionalGain = ntStsProportionalGain.getDouble(69.0);
    stsIntegralGain = ntStsIntegralGain.getDouble(69.0);
    stsDerivativeGain = ntStsDerivativeGain.getDouble(69.0);

    stsShooterUpperShooterHighSpeed = ntScdUpperShooterHighSpeed.getDouble(69.0);
    stsShooterLowerShooterHighSpeed = ntScdLowerShooterHighSpeed.getDouble(69.0);
    stsShooterUpperShooterLowSpeed = ntScdUpperShooterLowSpeed.getDouble(69.0);
    stsShooterLowerShooterLowSpeed = ntScdLowerShooterLowSpeed.getDouble(69.0);
  }

  public void PIDCalibrations()
  {
    if(ntScdUpdateProportionalGain.getBoolean(false))
    {
      stsProportionalGain = ntScdProportionalGain.getDouble(69.0);
      ntStsProportionalGain.forceSetDouble(stsProportionalGain);
      ntScdUpdateProportionalGain.setBoolean(false);
      topShooterTalonSRX.config_kP(0, stsProportionalGain);
      bottomShooterTalonSRX.config_kP(0, stsProportionalGain);
    }
    
    if(ntScdUpdateIntegralGain.getBoolean(false))
    {
      stsIntegralGain = ntScdIntegralGain.getDouble(69.0);
      ntStsIntegralGain.forceSetDouble(stsIntegralGain);
      ntScdUpdateIntegralGain.setBoolean(false);
      topShooterTalonSRX.config_kI(0, stsIntegralGain);
      bottomShooterTalonSRX.config_kI(0, stsIntegralGain);
    }

    if(ntScdUpdateDerivativeGain.getBoolean(false))
    {
      stsDerivativeGain = ntScdDerivativeGain.getDouble(69.0);
      ntStsDerivativeGain.forceSetDouble(stsDerivativeGain);
      ntScdUpdateDerivativeGain.setBoolean(false);
      topShooterTalonSRX.config_kD(0, stsDerivativeGain);
      bottomShooterTalonSRX.config_kD(0, stsDerivativeGain);
    }

    if(ntScdUpdateUpperShooterHighSpeed.getBoolean(false))
    {
      stsShooterUpperShooterHighSpeed = ntScdUpperShooterHighSpeed.getDouble(69.0);
      ntStsUpperShooterHighSpeed.forceSetDouble(stsShooterUpperShooterHighSpeed);
      ntScdUpperShooterHighSpeed.setBoolean(false);
    }

    if(ntScdUpdateLowerShooterHighSpeed.getBoolean(false))
    {
      stsShooterLowerShooterHighSpeed = ntScdLowerShooterHighSpeed.getDouble(69.0);
      ntStsLowerShooterHighSpeed.forceSetDouble(stsShooterLowerShooterHighSpeed);
      ntScdLowerShooterHighSpeed.setBoolean(false);
    }

    if(ntScdUpdateUpperShooterLowSpeed.getBoolean(false))
    {
      stsShooterUpperShooterLowSpeed = ntScdUpperShooterLowSpeed.getDouble(69.0);
      ntStsUpperShooterLowSpeed.forceSetDouble(stsShooterUpperShooterLowSpeed);
      ntScdUpperShooterLowSpeed.setBoolean(false);
    }

    if(ntScdUpdateLowerShooterLowSpeed.getBoolean(false))
    {
      stsShooterLowerShooterLowSpeed = ntScdLowerShooterLowSpeed.getDouble(69.0);
      ntStsLowerShooterLowSpeed.forceSetDouble(stsShooterLowerShooterLowSpeed);
      ntScdLowerShooterLowSpeed.setBoolean(false);
    }
  }

    public void setShooterSpeed()
    {
      if(driverJoystick.getRawAxis(2) > 0.1)
      {
        if(driverJoystick.getRawAxis(2) >= 0.51)
        {
          shooterHighSpeed = true;
        }
        else
        {
          shooterLowSpeed = true;
        }
      }
    }

    public boolean getShooterHighSpeed()
    {
      return shooterHighSpeed;
    }

    public boolean getShooterLowSpeed()
    {
      return shooterLowSpeed;
    }

 /* public void testShooter()
  {
    if(ntEnableCmdShooterShoot.getBoolean(false))
    {
      if(cmdEnableShooterShoot)
      {
        runCmdShooterShoot();
      }
    }
  }*/
}