/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.BallIndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CloseShooter extends CommandBase
{
  /**
   * Creates a new ShooterCommand.
   */
  ShooterSubsystem subsystem;
  double targetRPM;
  BallIndexerSubsystem indexerSubsystem;
  double targetServoSpeed;
  IntakeSubsystem intakeSub;

  public CloseShooter(ShooterSubsystem subsystem, IntakeSubsystem intakeSub, double targetRPM, BallIndexerSubsystem indexerSubsystem, double targetServoSpeed)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);// , indexerSubsystem);
    this.subsystem = subsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.targetRPM = targetRPM;
    this.targetServoSpeed = targetServoSpeed;
    this.intakeSub= intakeSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(targetRPM!=0)
    {
      subsystem.setVelocity(targetRPM);
    }
    else
    {
      subsystem.setPercentOutput(0);
    }
    intakeSub.setIntakeSolinoid(true);
    //subsystem.setServoSpeed(targetServoSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(subsystem.getDegrees());
    // if((subsystem.getVelocity() >= targetRPM-100) && (subsystem.getVelocity() >=
    // targetRPM+100)){
    // indexerSubsystem.setPercentOutput(1);
    // };
    subsystem.setHoodAngle(0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setPercentOutput(0);
    subsystem.setServoSpeed(0);
    intakeSub.setIntakeSolinoid(false);

    // indexerSubsystem.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return false; }
}
