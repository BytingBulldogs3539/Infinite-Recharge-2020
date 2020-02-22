/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autons;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.autonCommands.AutoAlignCommand;
import frc.robot.autonCommands.AutoShooterCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup
{
  /**
   * Creates a new Shoot.
   */
  public Shoot()
  {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new AutoAlignCommand(RobotContainer.m_robotDrive),
        // TrajectoryCommandGenerator.getMotionCommand(new Pose2d(0, 0,
        // Rotation2d.fromDegrees(0)), null,
        // new Pose2d(-60, 0, Rotation2d.fromDegrees(0)),
        // true));
        new AutoShooterCommand(RobotContainer.m_ShooterSubsystem, 5700, RobotContainer.m_BallIndexerSubsystem,
            RobotContainer.m_robotDrive, 10).withTimeout(5));

  }
}
