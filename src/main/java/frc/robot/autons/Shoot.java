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
import frc.robot.commands.BallIndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.BallIndexerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup {
  /**
   * Creates a new Shoot.
   */
  public Shoot(DriveSubsystem driveSub, IntakeSubsystem intakeSub, ShooterSubsystem shooterSub, BallIndexerSubsystem ballIndexerSubsystem)
  {  // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
        new AutoAlignCommand(driveSub).andThen(() ->driveSub.drive(0, 0, 0, true)),

         new AutoShooterCommand(shooterSub, 5000,
         ballIndexerSubsystem,
         driveSub, false).withTimeout(3),
        new ParallelCommandGroup(
        TrajectoryCommandGenerator.getMotionCommand(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), List.of(),
            new Pose2d(-69, -22, Rotation2d.fromDegrees(0)), true, driveSub),
            new IntakeCommand(intakeSub,true).withTimeout(2)
            ),

        new ParallelCommandGroup(
            TrajectoryCommandGenerator.getMotionCommand(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), List.of(),
                new Pose2d(-100, 0, Rotation2d.fromDegrees(0)), true, driveSub),
            new IntakeCommand(intakeSub,true).withTimeout(2),
            new BallIndexerCommand(ballIndexerSubsystem).withTimeout(2)),

        new ParallelCommandGroup(
            TrajectoryCommandGenerator.getMotionCommand(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), List.of(),
                new Pose2d(153, 25, Rotation2d.fromDegrees(0)), false, driveSub),
            new IntakeCommand(intakeSub,false).withTimeout(2),
            new BallIndexerCommand(ballIndexerSubsystem).withTimeout(2),
            new AutoShooterCommand(shooterSub, 5000, ballIndexerSubsystem, driveSub, true).withTimeout(3)
            ),

        new ParallelCommandGroup(
            new AutoAlignCommand(driveSub).andThen(() ->driveSub.drive(0, 0, 0, true)),
        new AutoShooterCommand(shooterSub, 5000, ballIndexerSubsystem, driveSub, true).withTimeout(1)),

        new AutoShooterCommand(shooterSub,5000, ballIndexerSubsystem,
            driveSub,false).withTimeout(5)
    );


  }
}
