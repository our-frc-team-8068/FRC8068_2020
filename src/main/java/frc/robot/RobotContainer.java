/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CmdDefaultCollector;
import frc.robot.commands.CmdDefaultControlPanel;
import frc.robot.commands.CmdDefaultJoystickDrive;
import frc.robot.commands.CmdDefaultMagazinePosition;
import frc.robot.commands.CmdDefaultShoot;
import frc.robot.commands.CmdDriveTrainInvertDirection;
import frc.robot.commands.CmdMagazineHomeEncoder;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

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

  private XboxController testController = new XboxController(2);

  private final DigitalInput colorCalibrationEnabled = new DigitalInput(Constants.DIO_MagazineCollectorAllowColorCalibration);


  //Subsystems
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  private final Arm arm = new Arm(driverJoystick, operatorJoystick);
  private final Magazine magazine = new Magazine(driverJoystick);
  private final Collector collector = new Collector(driverJoystick, operatorJoystick, magazine);
  private final DriveTrain driveTrain = new DriveTrain(driverJoystick);
  private final Shooter shooter = new Shooter(driverJoystick, magazine);
  private final ControlPanel controlPanel = new ControlPanel(operatorJoystick, driveTrain);
  private final Vision vision = new Vision(driverJoystick, operatorJoystick);

  //Commands
  private final ExampleCommand autoCommand = new ExampleCommand(exampleSubsystem);
  private final CmdDefaultJoystickDrive cmdDefaultJoystickDrive = new CmdDefaultJoystickDrive(driverJoystick, driveTrain);
  private final CmdDefaultMagazinePosition cmdDefaultMagazinePosition = new CmdDefaultMagazinePosition(magazine);
  private final CmdMagazineHomeEncoder cmdMagazineHomeEncoder = new CmdMagazineHomeEncoder(magazine);
  private final CmdDefaultCollector cmdDefaultCollector = new CmdDefaultCollector(collector, driverJoystick, magazine);
  private final CmdDefaultShoot cmdDefaultShoot = new CmdDefaultShoot(shooter, magazine, driverJoystick);
  private final CmdDefaultControlPanel cmdDefaultControlPanel = new CmdDefaultControlPanel(operatorJoystick, controlPanel);

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
    controlPanel.setDefaultCommand(cmdDefaultControlPanel);

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

  public void runHomeMagazineCommand()
  {
    cmdMagazineHomeEncoder.schedule();
  }

}
