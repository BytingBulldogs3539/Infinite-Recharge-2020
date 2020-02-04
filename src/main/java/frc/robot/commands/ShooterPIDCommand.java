/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShooterPIDCommand extends PIDCommand {
  ShooterSubsystem subsystem;

  /**
   * Creates a new ShooterPIDCommand.
   */
  public ShooterPIDCommand(ShooterSubsystem subsystem) {
    super(
      // The controller that the command will use
      // TODO: tune/maybe add to robot constants(?)
      new PIDController(0.001, 0, 0),
      // This should return the measurement
      subsystem::getAnalog,
      // This should return the setpoint (can also be a constant)
      subsystem::getSetpoint,
      // This uses the output
      output -> {
        double out = MathUtil.clamp(output, -0.5, 0.5);
        out += 0.5;
        subsystem.setServoSpeed(out);
      }
    );
    getController().setTolerance(120);
    addRequirements(subsystem);
    this.subsystem = subsystem;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
