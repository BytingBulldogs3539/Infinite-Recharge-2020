/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  /**
   * Creates a new IntakeCommand.
   */
  IntakeSubsystem subsystem;

  public IntakeCommand(IntakeSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.setPercentOutput(RobotContainer.m_opController.getRawAxis(3)-(-RobotContainer.m_opController.getRawAxis(2)));
    //the Right Trigger (3) minus the negative Left Trigger (2), giving ranges from -1 to 1

    // subsystem.drive(RobotContainer.m_driverController.get);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
