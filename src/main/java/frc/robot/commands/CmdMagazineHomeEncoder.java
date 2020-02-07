/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;
import com.revrobotics.ColorMatch;
import frc.robot.Robot;

public class CmdMagazineHomeEncoder extends CommandBase {
  /**
   * Creates a new CmdMagazineHomeEncoder.
   */

  private final Magazine magazine;
  private final WPI_VictorSPX magazineVictorSPX;
  private final Encoder magazinePositionEncoder;
  private final ColorSensorV3 magazineHomePositionColorSensor;

  private final ColorMatch colorMatcher = new ColorMatch();

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  public CmdMagazineHomeEncoder(Magazine magazine, WPI_VictorSPX magazineVictorSPX, Encoder magazinePositionEncoder,
      ColorSensorV3 magazineHomePositionColorSensor, ColorMatch colorMatcher)
    {
      // Use addRequirements() here to declare subsystem dependencies.
      this.magazine = magazine;
      this.magazineHomePositionColorSensor = magazineHomePositionColorSensor;
      this.magazinePositionEncoder = magazinePositionEncoder;
      this.magazineVictorSPX = magazineVictorSPX;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
    {

    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
    {
      magazineVictorSPX.set(ControlMode.PercentOutput, 0.5);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
    {
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
    {
      return false;
    }
}
