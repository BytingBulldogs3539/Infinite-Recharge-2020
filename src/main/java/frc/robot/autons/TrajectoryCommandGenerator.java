/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autons;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.SwerveControllerCommand;

/**
 * Add your docs here.
 */
public class TrajectoryCommandGenerator
{
        public static Command getMotionCommand(Pose2d startPoint, List<Translation2d> interiorPoints,
                        Pose2d endPoint, boolean reverse, DriveSubsystem driveSub) {
                TrajectoryConfig config = RobotContainer.robotConstants.getDriveConstants().getTrajectoryConfig();
                 config.setReversed(reverse);
                // An example trajectory to follow. All units in inches.
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                startPoint,
                                // Pass through these two interior waypoints, making an 's' curve path
                                interiorPoints,
                                // new Pose2d(36,36, Rotation2d.fromDegrees(90)), config);
                                endPoint, config);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory,
                                driveSub::getPose, // Functional interface to feed supplier
                                RobotContainer.robotConstants.getDriveConstants().getKDriveKinematics(),

                                // Position controllers
                                new PIDController(RobotContainer.robotConstants.getAutoConstants().getKPXController(),
                                                0, RobotContainer.robotConstants.getAutoConstants().getKDXController()),
                                new PIDController(RobotContainer.robotConstants.getAutoConstants().getKPYController(),
                                                0, RobotContainer.robotConstants.getAutoConstants().getKDYController()),
                                new ProfiledPIDController(
                                                RobotContainer.robotConstants.getAutoConstants().getKPThetaController(),
                                                0, 0,
                                                RobotContainer.robotConstants.getAutoConstants()
                                                                .getKThetaControllerConstraints()),

                                driveSub::setModuleStates,

                                driveSub::getVisionSeeing,

                                driveSub::getVisionAngle,

                                false,

                                driveSub

                );

                // Run path following command, then stop at the end.
                return swerveControllerCommand.andThen(() -> {driveSub.drive(0, 0, 0, false);
                    driveSub.resetOdometry(new Pose2d(0,0,driveSub.getAngle()));});
        }

}
