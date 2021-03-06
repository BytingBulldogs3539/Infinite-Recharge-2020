/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallIndexerSubsystem;

public class BallIndexerCommand extends CommandBase
{
  /**
   * Creates a new BallIndexerCommand.
   */
  BallIndexerSubsystem subsystem;
  int counter;

  public BallIndexerCommand(BallIndexerSubsystem subsystem)
  {
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
    /*if (subsystem.getBallSensorState()){
      counter=5;
    }else{
      subsystem.setPercentOutput(0);
    }

    if(counter > 0){
      subsystem.setPercentOutput(1.0);
      counter--;
    }*/
    if(subsystem.getBallSensorState()){
      subsystem.setPercentOutput(1.0);
    }else{
      subsystem.setPercentOutput(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return false; }
}
