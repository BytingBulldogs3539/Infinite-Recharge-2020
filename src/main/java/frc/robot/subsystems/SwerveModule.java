/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.utilities.PIDController;
import frc.robot.utilities.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.RobotContainer;

public class SwerveModule {
  public String name;
  public final CANSparkMax m_driveMotor;
  public final CANSparkMax m_turningMotor;

  private final boolean driveEncoderReversed;

  private final CANEncoder m_driveEncoder;

  public final CANCoder m_turningEncoder;
  private final CANCoderConfiguration m_turningEncoderConfiguration = new CANCoderConfiguration();

  private final PIDController m_drivePIDController = new PIDController(RobotContainer.robotConstants.getModuleConstants().getkPModuleDriveController(), 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
    RobotContainer.robotConstants.getModuleConstants().getkPModuleTurningController(), 0, 0,
      new TrapezoidProfile.Constraints(RobotContainer.robotConstants.getModuleConstants().getkMaxModuleAngularSpeedRadiansPerSecond(),
      RobotContainer.robotConstants.getModuleConstants().getkMaxModuleAngularAccelerationRadiansPerSecondSquared()));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(String name, int driveMotorChannel, int turningMotorChannel, CANCoder turningEncoderPort, double magnetOffsetDegrees,
      boolean driveEncoderReversed, boolean turningEncoderReversed, boolean reverseDrive, boolean reverseTurn) {
    this.name = name;
    // Create two CANSparkMax Motor controllers assuming brushless mode that control
    // the turning and drive axies
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);


    // Invert the motors if needed.
    m_driveMotor.setInverted(reverseDrive);
    m_turningMotor.setInverted(reverseTurn);

    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_turningMotor.setIdleMode(IdleMode.kCoast);



    // Get and store away the built in encoder on the drive motor.
    this.m_driveEncoder = m_driveMotor.getEncoder();

    // Create and store away the turning motor.
    this.m_turningEncoder = turningEncoderPort;//new CANCoder(turningEncoderPort);

    // Set whether drive encoder should be reversed or not
    this.driveEncoderReversed = driveEncoderReversed;

    // Set whether turning encoder should be reversed or not.
    m_turningEncoderConfiguration.sensorDirection = turningEncoderReversed;
    m_turningEncoderConfiguration.magnetOffsetDegrees = magnetOffsetDegrees;
    m_turningEncoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    m_turningEncoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;

    this.m_turningEncoder.configAllSettings(m_turningEncoderConfiguration);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // Set the name of anything that might be added to the smartdashboard.
    // PID Controllers.
    SendableRegistry.setName(m_turningPIDController, String.format("%s %s", name, "Turning PID Controller"));
    SendableRegistry.setName(m_drivePIDController, String.format("%s %s", name, "Drive PID Controller"));

  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveVel() * RobotContainer.robotConstants.getModuleConstants().getkDriveEncoderRpmToInps(),
        new Rotation2d(getAngle()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.
    state = optimizeModuleAngle(state, getAngle());

    final var driveOutput = m_drivePIDController.calculate(getDriveVel() * RobotContainer.robotConstants.getModuleConstants().getkDriveEncoderRpmToInps(),
        state.speedMetersPerSecond);
    SmartDashboard.putNumber("Current Speed", getDriveVel()
        * RobotContainer.robotConstants.getModuleConstants().getkDriveEncoderRpmToInps());
    SmartDashboard.putNumber("Drive Speed", state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput = m_turningPIDController.calculate(getAngle(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput + RobotContainer.robotConstants.getModuleConstants().getkFModuleDriveController() * state.speedMetersPerSecond);
    m_turningMotor.set(turnOutput);
  }

  public void stopDrive() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }

  /**
   * 
   * Zeros all the SwerveModule encoders.
   */

  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
  }

  public double getDrivePos()
  {
    return m_driveMotor.getEncoder().getPosition()* ((driveEncoderReversed) ? -1 : 1);
  }
  public double getDriveVel()
  {
    return m_driveEncoder.getVelocity()* ((driveEncoderReversed) ? -1 : 1);
  }

  public double getAngle() {
    double enc = Math.toRadians(m_turningEncoder.getAbsolutePosition());
    // double angle = 0;
    // if (Math.sin(enc) < 0)
    // angle = -Math.acos(Math.cos(enc));
    // else
    // angle = Math.acos(Math.cos(enc));
    // return angle;
    return enc;
  }

  /**
   * Optimizes the angle of the module.
   *
   * @param desiredState The new state of the module.
   * @param angle        The current angle of the module in radians.
   */
  public static SwerveModuleState optimizeModuleAngle(SwerveModuleState desiredState, double angle) {
    SwerveModuleState finalState = new SwerveModuleState(desiredState.speedMetersPerSecond,
        new Rotation2d(desiredState.angle.getRadians()));
    Rotation2d deltaAngle = Rotation2d.fromDegrees(desiredState.angle.getDegrees() - Math.toDegrees(angle));
    if (Math.abs(deltaAngle.getDegrees()) > 90 && Math.abs(deltaAngle.getDegrees()) < 270) {
      finalState.angle = finalState.angle.plus(Rotation2d.fromDegrees(180));
      finalState.speedMetersPerSecond = -finalState.speedMetersPerSecond;
    }
    return finalState;
  }
}
