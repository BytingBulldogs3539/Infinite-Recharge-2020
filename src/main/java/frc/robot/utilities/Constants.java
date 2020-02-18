/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * An interface conta
 */
public abstract class Constants
{

    public abstract RobotIDConstants getRobotIDConstants();

    public abstract DriveConstants getDriveConstants();

    public abstract ModuleConstants getModuleConstants();

    public abstract OIConstants getOIConstants();

    public abstract AutoConstants getAutoConstants();

    public abstract IntakeConstants getIntakeConstants();

    public abstract BallIndexerConstants getBallIndexerConstants();

    public abstract ClimbConstants getClimbConstants();

    public abstract ShooterConstants getShooterConstants();

    public abstract class RobotIDConstants
    {
        public abstract int getIntakeMotorID();

        public abstract int getShooterMotorID();

        public abstract int getIndexMotorID();

        public abstract int getClimbMotorLID();

        public abstract int getClimbMotorRID();

        public abstract int getAdjusterID();

        public abstract int getBuddyClimbMotorID();

        public abstract int getSpinnerMotorID();

        public abstract I2C.Port getColorSensorPort();

        public abstract int getShooterPotentiometerID();

        public abstract int getShooterServoLID();
        public abstract int getShooterServoRID();

        public abstract int getBallSensorPort();

        public abstract int getPCMID();

        public abstract int getIntakeSolinoidOn();

        public abstract int getIntakeSolinoidOff();
    }

    public abstract class DriveConstants
    {
        public abstract int getKFrontLeftDriveMotorPort();

        public abstract int getKRearLeftDriveMotorPort();

        public abstract int getKFrontRightDriveMotorPort();

        public abstract int getKRearRightDriveMotorPort();

        public abstract int getKFrontLeftTurningMotorPort();

        public abstract int getKRearLeftTurningMotorPort();

        public abstract int getKFrontRightTurningMotorPort();

        public abstract int getKRearRightTurningMotorPort();

        public abstract int getKPigeonID();

        public abstract int getKFrontLeftTurningEncoderPorts();

        public abstract int getKRearLeftTurningEncoderPorts();

        public abstract int getKFrontRightTurningEncoderPorts();

        public abstract int getKRearRightTurningEncoderPorts();

        public abstract double getKFrontLeftTurningEncoderOffset();

        public abstract double getKRearLeftTurningEncoderOffset();

        public abstract double getKFrontRightTurningEncoderOffset();

        public abstract double getKRearRightTurningEncoderOffset();

        public abstract boolean getKFrontLeftTurningEncoderReversed();

        public abstract boolean getKRearLeftTurningEncoderReversed();

        public abstract boolean getKFrontRightTurningEncoderReversed();

        public abstract boolean getKRearRightTurningEncoderReversed();

        public abstract boolean getKFrontLeftDriveEncoderReversed();

        public abstract boolean getKRearLeftDriveEncoderReversed();

        public abstract boolean getKFrontRightDriveEncoderReversed();

        public abstract boolean getKRearRightDriveEncoderReversed();

        public abstract boolean getKFrontLeftDriveReversed();

        public abstract boolean getKRearLeftDriveReversed();

        public abstract boolean getKFrontRightDriveReversed();

        public abstract boolean getKRearRightDriveReversed();

        public abstract boolean getKFrontLeftTurningReversed();

        public abstract boolean getKRearLeftTurningReversed();

        public abstract boolean getKFrontRightTurningReversed();

        public abstract boolean getKRearRightTurningReversed();

        public abstract double getKTrackWidth();

        public abstract double getKWheelBase();

        public abstract SwerveDriveKinematics getKDriveKinematics();

        public abstract boolean getKGyroReversed();

        public abstract double getKMaxSpeedINPerSecond();

        public abstract double getKMaxTurnSpeedRadPerSecond();

    }

    public abstract class ModuleConstants
    {
        public abstract double getKMaxModuleAngularSpeedRadiansPerSecond();

        public abstract double getKMaxModuleAngularAccelerationRadiansPerSecondSquared();

        public abstract int getKDriveEncoderCPR();

        public abstract double getKWheelDiameterIN();

        public abstract double getKDriveEncoderDistancePerPulse();

        public abstract double getKDriveEncoderRpmToInps();

        public abstract double getKPModuleTurningController();

        public abstract double getKPModuleDriveController();

        public abstract double getKFModuleDriveController();

    }

    public abstract class OIConstants
    {
        public abstract int getKDriverControllerPort();

        public abstract int getKOpControllerPort();
    }

    public abstract class AutoConstants
    {
        public abstract double getKMaxSpeedINPerSecond();

        public abstract double getKMaxAccelerationINPerSecondSquared();

        public abstract double getKMaxAngularSpeedRadiansPerSecond();

        public abstract double getKMaxAngularSpeedRadiansPerSecondSquared();

        public abstract double getKPXController();

        public abstract double getKPYController();

        public abstract double getKDXController();

        public abstract double getKDYController();

        public abstract double getKPThetaController();

        public abstract TrapezoidProfile.Constraints getKThetaControllerConstraints();

    }

    public abstract class IntakeConstants
    {
        public abstract MotorType getIntakeMotorType();

        public abstract boolean getIntakeMotorInverted();

        public abstract boolean getDefaultIntakeDirection();

        /** The number of loops for the intake to run backwards */
        public abstract int intakeReverseDelay();
    }

    public abstract class BallIndexerConstants
    {
        public abstract boolean getIndexMotorInverted();

        public abstract boolean getIndexMotorBrake();
    }

    public abstract class ClimbConstants
    {
        public abstract boolean getClimbMotorInverted();

        public abstract boolean getClimbMotorBrake();

        public abstract boolean getClimbAdjusterMotorInverted();

        public abstract boolean getClimbAdjusterMotorBrake();

    }

    public abstract class ShooterConstants
    {
        public abstract double getKP();

        public abstract double getKI();

        public abstract double getKD();

        public abstract double getKF();

        public abstract double getPotOffset();

        public abstract boolean getShooterMotorInverted();

        public abstract double getHoodOffset();

        public abstract boolean invertHoodAngle();
    }
}
