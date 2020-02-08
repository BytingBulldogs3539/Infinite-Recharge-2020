/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterChirpCommand extends CommandBase {
  ShooterSubsystem subsystem;
  boolean play;
  int delay;
  public ShooterChirpCommand(ShooterSubsystem subsystem,boolean play,int delay) {
    this.subsystem = subsystem;
    this.play =play;
    this.delay = delay;

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    addRequirements(subsystem);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    
    subsystem.initChirp();
    Timer.delay(delay);
    subsystem.PlayPauseChirp(play);
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    subsystem.StopChirp();
  }


}
