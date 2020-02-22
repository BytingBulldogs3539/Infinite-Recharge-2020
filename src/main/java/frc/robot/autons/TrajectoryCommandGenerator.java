/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autons;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utilities.SwerveControllerCommand;

/**
 * Add your docs here.
 */
public class TrajectoryCommandGenerator
{
        public static Command getMotionCommand(Pose2d startPoint, ArrayList<Translation2d> interiorPoints,
                        Pose2d endPoint, boolean reverse) {
                // config.setReversed(true);
                // An example trajectory to follow. All units in inches.
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                startPoint,
                                // Pass through these two interior waypoints, making an 's' curve path
                                interiorPoints,
                                // new Pose2d(36,36, Rotation2d.fromDegrees(90)), config);
                                endPoint, RobotContainer.robotConstants.getDriveConstants().getTrajectoryConfig());

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory,
                                RobotContainer.m_robotDrive::getPose, // Functional interface to feed supplier
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

                                RobotContainer.m_robotDrive::setModuleStates,

                                RobotContainer.m_robotDrive::getVisionSeeing,

                                RobotContainer.m_robotDrive::getVisionAngle,

                                true,

                                RobotContainer.m_robotDrive

                );

                // Run path following command, then stop at the end.
                return swerveControllerCommand.andThen(() -> RobotContainer.m_robotDrive.drive(0, 0, 0, false));
        }

}
