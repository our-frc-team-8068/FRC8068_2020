/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CmdCollectorCollect;
import frc.robot.commands.CmdDefaultCollector;
import frc.robot.commands.CmdDefaultJoystickDrive;
import frc.robot.commands.CmdDefaultMagazinePosition;
import frc.robot.commands.CmdDefaultShoot;
import frc.robot.commands.CmdDriveTrainInvertDirection;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  //JoySticks
  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick operatorJoystick = new Joystick(1);

  //Example Subsystem
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

  //Arm Subsystem
  private final WPI_VictorSPX rotationVictorSPX = new WPI_VictorSPX(30);
  private final WPI_VictorSPX leftWinchVictorSPX = new WPI_VictorSPX(31);
  private final WPI_VictorSPX rightWinchVictorSPX= new WPI_VictorSPX(32);
  private final WPI_VictorSPX shimmyVictorSPX = new WPI_VictorSPX(33);
  private final Encoder armPositionEncoder = new Encoder(2,3);
  private final ADIS16470_IMU robotImu = new ADIS16470_IMU();

  //Collector Subsystem
  private final WPI_VictorSPX collectorVictorSPX = new WPI_VictorSPX(45);

  //ControlPanel Subsystem
  private final WPI_VictorSPX controlPanelVictorSPX = new WPI_VictorSPX(40);
  private final I2C.Port controlPanelColorSensorI2CPort = I2C.Port.kOnboard;
  private final ColorSensorV3 controlPanelColorSensor = new ColorSensorV3(controlPanelColorSensorI2CPort);

  //Drivetrain Subsystem
  private final WPI_TalonSRX leftDriveTalonSRX = new WPI_TalonSRX(11);
  private final WPI_TalonSRX rightDriveTalonSRX = new WPI_TalonSRX(10);
  private final WPI_VictorSPX leftDriveVictorSPX = new WPI_VictorSPX(13);
  private final WPI_VictorSPX rightDriveVictorSPX = new WPI_VictorSPX(12);

  //Magazine Subsystem


  //Shooter Subsystem
  private final WPI_VictorSPX preigniterVictorSPX = new WPI_VictorSPX(20);
  private final WPI_TalonSRX topShooterTalonSRX = new WPI_TalonSRX(21);
  private final WPI_TalonSRX bottomShooterTalonSRX = new WPI_TalonSRX(22);

  private final Arm arm = new Arm(rotationVictorSPX, leftWinchVictorSPX, rightWinchVictorSPX, shimmyVictorSPX, driverJoystick, operatorJoystick);
  private final Collector collector = new Collector(collectorVictorSPX, driverJoystick, operatorJoystick);
  private final DriveTrain driveTrain = new DriveTrain(leftDriveTalonSRX, rightDriveTalonSRX, leftDriveVictorSPX, rightDriveVictorSPX, driverJoystick);
  private final Magazine magazine = new Magazine(driverJoystick);
  private final Shooter shooter = new Shooter(preigniterVictorSPX, topShooterTalonSRX, bottomShooterTalonSRX, driverJoystick);

  private final ExampleCommand autoCommand = new ExampleCommand(exampleSubsystem);
  private final CmdDefaultJoystickDrive cmdDefaultJoystickDrive = new CmdDefaultJoystickDrive(driverJoystick, driveTrain);
  private final CmdDefaultMagazinePosition cmdDefaultMagazinePosition = new CmdDefaultMagazinePosition(magazine);
  private final CmdDefaultCollector cmdDefaultCollector = new CmdDefaultCollector(collector);
  private final CmdDefaultShoot cmdDefaultShoot = new CmdDefaultShoot(shooter);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveTrain.setDefaultCommand(cmdDefaultJoystickDrive);
    magazine.setDefaultCommand(cmdDefaultMagazinePosition);
    collector.setDefaultCommand(cmdDefaultCollector);
    shooter.setDefaultCommand(cmdDefaultShoot);
    //m_driveTrain.setDefaultCommand(new CmdDefaultJoystickDrive(m_driverJoystick, m_driveTrain));

    //m_drivetrain.setDefaultCommand(new TankDrive(() -> m_joystick.getY(Hand.kLeft),
    //    () -> m_joystick.getY(Hand.kRight), m_drivetrain));

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton collectorCollectButton = new JoystickButton(driverJoystick, 
      LogitechGamePad.LEFT_BUMPER);
    final JoystickButton invertDriveButton = new JoystickButton(driverJoystick, LogitechGamePad.BUTTON_Y);

    collectorCollectButton.whileHeld(new CmdCollectorCollect(magazine, collector));
    invertDriveButton.whenPressed(new CmdDriveTrainInvertDirection(driveTrain));

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoCommand;
  }
}
