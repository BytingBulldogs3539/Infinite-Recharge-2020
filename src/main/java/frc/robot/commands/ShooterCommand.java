/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallIndexerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase
{
  /**
   * Creates a new ShooterCommand.
   */
  ShooterSubsystem subsystem;
  double targetRPM;
  BallIndexerSubsystem indexerSubsystem;
  DriveSubsystem driveSub;
  double targetServoSpeed;

  public ShooterCommand(ShooterSubsystem subsystem, double targetRPM, BallIndexerSubsystem indexerSubsystem, DriveSubsystem driveSub,double targetServoSpeed)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);// , indexerSubsystem);
    this.subsystem = subsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.targetRPM = targetRPM;
    this.targetServoSpeed = targetServoSpeed;
    this.driveSub = driveSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(targetRPM != 0)
    {
      subsystem.setVelocity(targetRPM);
    }
    else
    {
      subsystem.setPercentOutput(0);
    }
    subsystem.setHoodAngle(-.0055*Math.pow((driveSub.getTargetHeight()-70),2)+12);
    //subsystem.setServoSpeed(targetServoSpeed);
    //subsystem.setHoodAngle(20);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     //System.out.println(subsystem.getDegrees());
     //System.out.println(subsystem.getVelocity());
    //  if((subsystem.getVelocity() >= targetRPM-200) && (subsystem.getVelocity() <=
    //  targetRPM+200) && targetRPM!=0){
    //  indexerSubsystem.setPercentOutput(1);
    //  };
    
    //subsystem.setHoodAngle(SmartDashboard.getNumber("Servo Target", 30));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setPercentOutput(0);
    subsystem.setServoSpeed(0);
    // indexerSubsystem.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return false; }
}
