/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase
{
  /**
   * Creates a new IntakeCommand.
   */
  IntakeSubsystem subsystem;
  /**Used to count loops in the intake command*/
  int counter = 0;
  boolean shouldRunBack;

  public IntakeCommand(IntakeSubsystem subsystem, boolean shouldRunBack)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.subsystem = subsystem;
    this.shouldRunBack = shouldRunBack;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    if(this.shouldRunBack)
      subsystem.setPercentOutput(-1);
    subsystem.setIntakeSolinoid(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(this.shouldRunBack)
    {
      if(counter<RobotContainer.robotConstants.getIntakeConstants().intakeReverseDelay()){
        counter++;
      }else{
        subsystem.setPercentOutput(1);
      }
    }
    else{
      subsystem.setPercentOutput(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setIntakeSolinoid(false);
    subsystem.setPercentOutput(0);
    counter = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return false; }
}
