/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbAdjustCommand extends CommandBase
{
  /**
   * Creates a new ClimbAdjustCommand.
   */
  double percentOutput;
  ClimbSubsystem subsystem;

  public ClimbAdjustCommand(ClimbSubsystem subsystem, double percentOutput)
  {
    addRequirements(subsystem);
    this.subsystem = subsystem;
    this.percentOutput = percentOutput;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.setAdjusterPercentOutput(percentOutput);
    System.out.println("Running");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setAdjusterPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return false; }
}
