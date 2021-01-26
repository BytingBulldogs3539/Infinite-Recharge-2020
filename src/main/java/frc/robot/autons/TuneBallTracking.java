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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.autonCommands.AutoAlignCommand;
import frc.robot.autonCommands.AutoShooterCommand;
import frc.robot.autonCommands.TrajectoryCommandGenerator;
import frc.robot.commands.BallIndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.BallIndexerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TuneBallTracking extends SequentialCommandGroup {
    /**
     * Creates a new Shoot.
     */
    public TuneBallTracking(DriveSubsystem driveSub, IntakeSubsystem intakeSub, ShooterSubsystem shooterSub,
            BallIndexerSubsystem ballIndexerSubsystem) { // Add your commands in the super() call, e.g.
                                                         // super(new FooCommand(), new BarCommand());
        super(
        new ParallelRaceGroup(
            TrajectoryCommandGenerator.getMotionCommand(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), List.of(),
                new Pose2d(-30, 0, Rotation2d.fromDegrees(0)), .1 ,false,true,true, driveSub))
    );
              

  }
}
