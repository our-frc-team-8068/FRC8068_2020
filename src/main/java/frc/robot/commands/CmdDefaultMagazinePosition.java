/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Magazine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CmdDefaultMagazinePosition extends PIDCommand {
  /**
   * Creates a new CmdPositionMagazine.
   */
  private final Magazine magazine;

  public CmdDefaultMagazinePosition(Magazine magazine) {
    super(
      // The controller that the command will use
      new PIDController(0.001, 0, 0),
      // This should return the measurement
      () -> magazine.getMagazineEncoderValue(),
      // This should return the setpoint (can also be a constant)
      () -> magazine.getSetpointEncoderCounts(),
      // This uses the output
      output -> {
        // Use the output here
        magazine.setMagazineSpeed(output);
      });
      this.magazine = magazine;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(magazine);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
