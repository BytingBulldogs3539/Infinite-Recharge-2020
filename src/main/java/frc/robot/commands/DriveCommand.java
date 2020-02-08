/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase
{
  /**
   * Creates a new DriveCommand.
   */
  DriveSubsystem subsystem;

  public DriveCommand(DriveSubsystem subsystem)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.subsystem = subsystem;
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

    subsystem.drive(RobotContainer.m_driverController.getLeftStickY(),
        -RobotContainer.m_driverController.getLeftStickX(), -RobotContainer.m_driverController.getRightStickX(), true);
    // System.out.println("Y1:"+RobotContainer.m_driverController.getLeftStickY()+"X:1"+-RobotContainer.m_driverController.getLeftStickX()+"X2:
    // "+-RobotContainer.m_driverController.getRightStickX());
    System.out.println(subsystem.getPose());

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
