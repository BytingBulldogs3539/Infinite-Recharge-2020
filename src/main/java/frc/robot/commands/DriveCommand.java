/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

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
  public void initialize() {
    System.out.println("DRIVE INIT");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = -RobotContainer.m_driverController.getLeftStickX();
    double ySpeed = RobotContainer.m_driverController.getLeftStickY();
    double rot = -RobotContainer.m_driverController.getRightStickX();

    
    if (xSpeed >= .09 || xSpeed <= -.09)
    xSpeed = xSpeed * .5;
    else
    xSpeed=0;
    if (ySpeed >= .09 || ySpeed <= -.09)
    ySpeed = ySpeed * .5;
    else
    ySpeed = 0;
      if (rot >= .09 || rot <= -.09)
      rot = rot * .5;
      else
      rot=0;
      if(RobotContainer.m_driverController.buttonBR.get())
      {
        subsystem.drive(ySpeed,
        xSpeed, rot, false);
      }
      else
      {
        subsystem.drive(ySpeed,
        xSpeed, rot, true);
      }
    
    // System.out.println("Y1:"+RobotContainer.m_driverController.getLeftStickY()+"X:1"+-RobotContainer.m_driverController.getLeftStickX()+"X2:
    // "+-RobotContainer.m_driverController.getRightStickX());
    // System.out.println(subsystem.getPose());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return false; }
}
