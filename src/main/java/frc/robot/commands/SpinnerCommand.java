/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SpinnerSubsystem;

public class SpinnerCommand extends CommandBase
{
  /**
   * Creates a new SpinnerCommand.
   */
  SpinnerSubsystem subsystem;

  public SpinnerCommand(SpinnerSubsystem subsystem)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.resetRotations();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int backSpin = 0;
    boolean spinCycle = true;
    // If FMS has sent a color, spin to that color:
    if(DriverStation.getInstance().getGameSpecificMessage().length() > 0 && subsystem.getNumColor() != subsystem.getGoalColor(DriverStation.getInstance().getGameSpecificMessage().charAt(0))) {
      if(subsystem.getNumColor() == 1 && subsystem.getGoalColor(DriverStation.getInstance().getGameSpecificMessage().charAt(0)) == 4){
        //Red to "Green" Yellow (1 to 4)
        subsystem.setPercentOutput(-0.3);
      }else if(subsystem.getNumColor() == 4 && subsystem.getGoalColor(DriverStation.getInstance().getGameSpecificMessage().charAt(0)) == 1){
        //Green to "Red" Blue (4 to 1)
        subsystem.setPercentOutput(0.3);
      }else if(subsystem.getNumColor() < subsystem.getGoalColor(DriverStation.getInstance().getGameSpecificMessage().charAt(0))){
        //If the current color is less than the goal color
        subsystem.setPercentOutput(0.3);
        backSpin = 5;
        spinCycle = true;
      }else if(subsystem.getNumColor() > subsystem.getGoalColor(DriverStation.getInstance().getGameSpecificMessage().charAt(0))){
         //If the current color is greater than the goal color
        subsystem.setPercentOutput(-0.3);
        backSpin = 5;
        spinCycle = false;
      }else{
         //What happened
        subsystem.setPercentOutput(0);
      }
    // If 3-5 rotations are required, rotate at least 4 times:
    } else if (DriverStation.getInstance().getGameSpecificMessage().length() == 0 && subsystem.getRotations() < 4) {
      subsystem.setPercentOutput(.75);
      subsystem.updateRoations();
      System.out.println(subsystem.getRotations());
    //If the color sensor just finished, run the motor backwards for a few loops.
    }else {
      subsystem.setPercentOutput(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop rotating just in case:
    subsystem.setPercentOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return false; }
}
