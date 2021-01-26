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

public class ShooterCommand extends CommandBase
{
  /**
   * Creates a new ShooterCommand.
   */
  ShooterSubsystem subsystem;
  double targetRPM;
  BallIndexerSubsystem indexerSubsystem;
  double targetServoSpeed;
  IntakeSubsystem intakeSub;

  public ShooterCommand(ShooterSubsystem subsystem, IntakeSubsystem intakeSub, double targetRPM, BallIndexerSubsystem indexerSubsystem, double targetServoSpeed)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem , indexerSubsystem);
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
    //System.out.println(subsystem.getDegrees());

    double velocity = subsystem.getVelocity();

    if((velocity >= targetRPM-175) && (velocity <=targetRPM+100))
    {
      indexerSubsystem.setPercentOutput(1);
      System.out.println("FEEEEEED");
    }
    else
      indexerSubsystem.setPercentOutput(0);

    SmartDashboard.putNumber("Shooter Velocity", velocity);
    SmartDashboard.putNumber("Shooter Target Velocity", targetRPM);
    double x = Robot.m_robotContainer.m_robotDrive.getTargetHeight();
    subsystem.setHoodAngle((.0000709536*Math.pow(x,3)-.0167058*Math.pow(x,2)+1.11431*x+4.21954));
    // System.out.println("X "+x);
    // System.out.println("CUB " + .0000709536*Math.pow(x,3));
    // System.out.println("SQR "+.0167058*Math.pow(x,2));
    // System.out.println("LIN "+1.11431*x);
    //subsystem.setHoodAngle(-.0033*Math.pow((Robot.m_robotContainer.m_robotDrive.getTargetHeight()-44),2)+27);
    //subsystem.setHoodAngle(SmartDashboard.getNumber("Hood Angle", 0));
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
