/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonCommands;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.utilities.PIDController;
import frc.robot.utilities.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoAlignCommand extends PIDCommand {
    /**
     * Creates a new VisionTrack.
     */

    DriveSubsystem subsystem;

    public AutoAlignCommand(DriveSubsystem subsystem)
  {
    super(
        // The controller that the command will use
        new PIDController(1.0, 0.01, .05),
        // This should return the measurement
        () -> {System.out.println(subsystem.getVisionAngle());
            return subsystem.getVisionAngle();},
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output ->
        {
          // Use the output here
          //System.out.println(output);
          subsystem.drive(0,
              0,output, true);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(subsystem);
    this.subsystem = subsystem;
    getController().setIntegratorRange(-1, 1);
    getController().setIntegratorZone(2);
    getController().setTolerance(.02);
  }
 @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
    subsystem.drive(0, 0, 0, false);
  }
  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
    subsystem.drive(0, 0, 0, false);


  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return getController().atSetpoint(); }
}
