/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  public final SwerveModule m_frontLeft = new SwerveModule("Front Left Module", RobotContainer.robotConstants.getDriveConstants().getkFrontLeftDriveMotorPort(),
  RobotContainer.robotConstants.getDriveConstants().getkFrontLeftTurningMotorPort(), RobotContainer.robotConstants.getDriveConstants().getkFrontLeftTurningEncoderPorts(), RobotContainer.robotConstants.getDriveConstants().getkFrontLeftTurningEncoderOffset(),
  RobotContainer.robotConstants.getDriveConstants().getkFrontLeftDriveEncoderReversed(), RobotContainer.robotConstants.getDriveConstants().getkFrontLeftTurningEncoderReversed(),
  RobotContainer.robotConstants.getDriveConstants().getkFrontLeftDriveReversed(), RobotContainer.robotConstants.getDriveConstants().getkFrontLeftTurningReversed());

  public final SwerveModule m_rearLeft = new SwerveModule("Back Left Module", RobotContainer.robotConstants.getDriveConstants().getkRearLeftDriveMotorPort(),
  RobotContainer.robotConstants.getDriveConstants().getkRearLeftTurningMotorPort(),RobotContainer.robotConstants.getDriveConstants().getkRearLeftTurningEncoderPorts(), RobotContainer.robotConstants.getDriveConstants().getkRearLeftTurningEncoderOffset(),
      RobotContainer.robotConstants.getDriveConstants().getkRearLeftDriveEncoderReversed(), RobotContainer.robotConstants.getDriveConstants().getkRearLeftTurningEncoderReversed(),
      RobotContainer.robotConstants.getDriveConstants().getkRearLeftDriveReversed(), RobotContainer.robotConstants.getDriveConstants().getkRearLeftTurningReversed());

  public final SwerveModule m_frontRight = new SwerveModule("Front Right Module",
  RobotContainer.robotConstants.getDriveConstants().getkFrontRightDriveMotorPort(), RobotContainer.robotConstants.getDriveConstants().getkFrontRightTurningMotorPort(),
  RobotContainer.robotConstants.getDriveConstants().getkFrontRightTurningEncoderPorts(), RobotContainer.robotConstants.getDriveConstants().getkFrontRightTurningEncoderOffset(), RobotContainer.robotConstants.getDriveConstants().getkFrontRightDriveEncoderReversed(),
      RobotContainer.robotConstants.getDriveConstants().getkFrontRightTurningEncoderReversed(), RobotContainer.robotConstants.getDriveConstants().getkFrontRightDriveReversed(),
      RobotContainer.robotConstants.getDriveConstants().getkFrontRightTurningReversed());

  public final SwerveModule m_rearRight = new SwerveModule("Back Right Module",
  RobotContainer.robotConstants.getDriveConstants().getkRearRightDriveMotorPort(), RobotContainer.robotConstants.getDriveConstants().getkRearRightTurningMotorPort(),
  RobotContainer.robotConstants.getDriveConstants().getkRearRightTurningEncoderPorts(), RobotContainer.robotConstants.getDriveConstants().getkRearRightTurningEncoderOffset(), RobotContainer.robotConstants.getDriveConstants().getkRearRightDriveEncoderReversed(),
      RobotContainer.robotConstants.getDriveConstants().getkRearRightTurningEncoderReversed(), RobotContainer.robotConstants.getDriveConstants().getkRearRightDriveReversed(),
      RobotContainer.robotConstants.getDriveConstants().getkRearRightTurningReversed());

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  public static final NetworkTableInstance table = NetworkTableInstance.getDefault();
  public static final NetworkTable myCam = table.getTable("chameleon-vision").getSubTable("Microsoft LifeCam HD-3000");

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    Robot.m_robotContainer.m_BallIndexerSubsystem.pigeon.setYaw(0);
    m_odometry = new SwerveDriveOdometry(RobotContainer.robotConstants.getDriveConstants().getkDriveKinematics(), getAngle());
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(getPigeonAngle());  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(new Rotation2d(Math.toRadians(getHeading())), m_frontLeft.getState(), m_rearLeft.getState(),
        m_frontRight.getState(), m_rearRight.getState());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, getAngle());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (xSpeed >= .02 || xSpeed <= -.02)
      xSpeed = xSpeed * .5;
    else
      xSpeed = 0;

    if (ySpeed >= .02 || ySpeed <= -.02)
      ySpeed = ySpeed * .5;
    else
      ySpeed = 0;

    if (rot >= .02 || rot <= -.02)
      rot = rot * .5;
    else
      rot = 0;

    if (xSpeed == 0 & ySpeed == 0 && rot == 0) {
      stopDrive();
    } else {
      xSpeed *= RobotContainer.robotConstants.getDriveConstants().getkMaxSpeedINPerSecond();
      ySpeed *= RobotContainer.robotConstants.getDriveConstants().getkMaxSpeedINPerSecond();
      rot *= RobotContainer.robotConstants.getDriveConstants().getkMaxTurnSpeedRadPerSecond();
      var swerveModuleStates = RobotContainer.robotConstants.getDriveConstants().getkDriveKinematics()
          .toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getAngle())
              : new ChassisSpeeds(xSpeed, ySpeed, rot));
      setModuleStates(swerveModuleStates);
    }

  }

  public void stopDrive() {
    m_frontLeft.stopDrive();
    m_frontRight.stopDrive();
    m_rearLeft.stopDrive();
    m_rearRight.stopDrive();
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, RobotContainer.robotConstants.getDriveConstants().getkMaxSpeedINPerSecond());
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    Robot.m_robotContainer.m_BallIndexerSubsystem.pigeon.setYaw(0);
  }

  public double getPigeonAngle()
  {

    double[] ypr = new double[3];
    RobotContainer.m_BallIndexerSubsystem.pigeon.getYawPitchRoll(ypr);
    return ypr[0] * (RobotContainer.robotConstants.getDriveConstants().getkGyroReversed() ? -1.0 : 1.0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    
    return Math.IEEEremainder(getPigeonAngle(), 360);
  }

  public boolean getVisionSeeing()
  {
    if(myCam.getEntry("isValid").getBoolean(false))
    {
      return true;
    } 
    return false;
  }

  public double getVisionAngle()
  {
    double value = Math.toRadians(myCam.getEntry("targetYaw").getDouble(0));
    return value;
  }

  // The pigeon does not return a rate and it does'nt seem to need it...
  // /**
  // * Returns the turn rate of the robot.
  // *
  // * @return The turn rate of the robot, in degrees per second
  // */
  // public double getTurnRate() {
  // return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  // }
}
