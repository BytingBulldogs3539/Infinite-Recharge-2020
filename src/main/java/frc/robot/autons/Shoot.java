/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autons;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.autonCommands.AutoAlignCommand;
import frc.robot.autonCommands.AutoShooterCommand;
import frc.robot.commands.IntakeCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup {
  /**
   * Creates a new Shoot.
   */
  public Shoot() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super( //new AutoAlignCommand(RobotContainer.m_robotDrive).andThen(() ->
           //RobotContainer.m_robotDrive.drive(0, 0, 0, false)),

         //new AutoShooterCommand(RobotContainer.m_ShooterSubsystem, 5200,
         //RobotContainer.m_BallIndexerSubsystem,
         //RobotContainer.m_robotDrive, 10).withTimeout(5).andThen(() ->
         //RobotContainer.m_robotDrive.drive(0, 0, 0, false)),

        TrajectoryCommandGenerator.getMotionCommand(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), List.of(),
            new Pose2d(-74, -60, Rotation2d.fromDegrees(0)), true),

        new ParallelCommandGroup(
            TrajectoryCommandGenerator.getMotionCommand(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), List.of(),
                new Pose2d(-80, 0, Rotation2d.fromDegrees(0)), true),
            new IntakeCommand(RobotContainer.m_IntakeSubsystem).withTimeout(2)),

            TrajectoryCommandGenerator.getMotionCommand(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), List.of(),
            new Pose2d(143, 60, Rotation2d.fromDegrees(0)), false)
    );
      //
         
        //new AutoShooterCommand(RobotContainer.m_ShooterSubsystem, 5200, RobotContainer.m_BallIndexerSubsystem,
            //RobotContainer.m_robotDrive, 10).withTimeout(5));

  }
}
