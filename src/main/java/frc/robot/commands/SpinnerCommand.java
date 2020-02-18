/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SpinnerSubsystem;

public class SpinnerCommand extends CommandBase
{
  /**
   * Creates a new SpinnerCommand.
   */
  SpinnerSubsystem subsystem;

  public SpinnerCommand(SpinnerSubsystem subsystem)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.resetRotations();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If FMS has sent a color, spin to that color:
    if (DriverStation.getInstance().getGameSpecificMessage().length() > 0 && subsystem.getOffsetColor() != DriverStation.getInstance().getGameSpecificMessage().charAt(0)) {
      subsystem.setPercentOutput(1.0);
    // If 3-5 rotations are required, rotate at least 4 times:
    } else if (DriverStation.getInstance().getGameSpecificMessage().length() == 0 && subsystem.getRotations() < 4) {
      subsystem.setPercentOutput(1.0);
      subsystem.updateRoations();
      System.out.println(subsystem.getRotations());
    // Otherwise, stop rotating:
    } else {
      subsystem.setPercentOutput(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop rotating just in case:
    subsystem.setPercentOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return false; }
}
