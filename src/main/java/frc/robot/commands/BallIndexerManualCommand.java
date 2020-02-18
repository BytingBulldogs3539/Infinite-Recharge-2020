/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallIndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class BallIndexerManualCommand extends CommandBase
{
  /**
   * Creates a new BallIndexerCommand.
   */
  BallIndexerSubsystem subsystem;
  double percentOutput;
  ShooterSubsystem shooterSub;

  public BallIndexerManualCommand(BallIndexerSubsystem subsystem, ShooterSubsystem shooterSub,double percentOutput)
  {
    addRequirements(subsystem);
    this.subsystem = subsystem;
    this.percentOutput = percentOutput;
    this.shooterSub = shooterSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.setPercentOutput(percentOutput);
    if(percentOutput < 0)
      shooterSub.setPercentOutput(-1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setPercentOutput(0);
    if(percentOutput < 0)
      shooterSub.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return false; }
}
