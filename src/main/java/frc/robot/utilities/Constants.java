/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * An interface conta
 */
public abstract class Constants {

    private RobotIDConstants robotIDConstants;
    private DriveConstants robotDriveConstants;
    private ModuleConstants robotModuleConstants;
    private OIConstants robotOIConstants;
    private AutoConstants robotAutoConstants;
    private ShooterConstants robotShooterConstants;

    public abstract RobotIDConstants getRobotIDConstants();

    public abstract DriveConstants getDriveConstants();

    public abstract ModuleConstants getModuleConstants();

    public abstract OIConstants getOIConstants();

    public abstract AutoConstants getAutoConstants();

    public abstract ShooterConstants getShooterConstants();

    public abstract class RobotIDConstants {
        public abstract int getIntakeMotorID();

        public abstract int getElevatorBaseMotorID();
        public abstract int getElevatorLiftMotorID();

        public abstract int getShooterMotorID();
        public abstract int getIndexMotorID();

        public abstract int getClimbMotorAID();
        public abstract int getClimbMotorBID();

        public abstract int getBuddyClimbMotorID();
        
        public abstract int getSpinnerMotorID();

        public abstract I2C.Port getColorSensorPort();

        public abstract int getShooterPotentiometerID();
        public abstract int getShooterServoID();
    }

    public abstract class DriveConstants {
        public abstract int getkFrontLeftDriveMotorPort();

        public abstract int getkRearLeftDriveMotorPort();

        public abstract int getkFrontRightDriveMotorPort();

        public abstract int getkRearRightDriveMotorPort();

        public abstract int getkFrontLeftTurningMotorPort();

        public abstract int getkRearLeftTurningMotorPort();

        public abstract int getkFrontRightTurningMotorPort();

        public abstract int getkRearRightTurningMotorPort();

        public abstract int getkPigeonID();

        public abstract int[] getkFrontLeftTurningEncoderPorts();

        public abstract int[] getkRearLeftTurningEncoderPorts();

        public abstract int[] getkFrontRightTurningEncoderPorts();

        public abstract int[] getkRearRightTurningEncoderPorts();

        public abstract boolean getkFrontLeftTurningEncoderReversed();

        public abstract boolean getkRearLeftTurningEncoderReversed();

        public abstract boolean getkFrontRightTurningEncoderReversed();

        public abstract boolean getkRearRightTurningEncoderReversed();

        public abstract boolean getkFrontLeftDriveEncoderReversed();

        public abstract boolean getkRearLeftDriveEncoderReversed();

        public abstract boolean getkFrontRightDriveEncoderReversed();

        public abstract boolean getkRearRightDriveEncoderReversed();

        public abstract boolean getkFrontLeftDriveReversed();

        public abstract boolean getkRearLeftDriveReversed();

        public abstract boolean getkFrontRightDriveReversed();

        public abstract boolean getkRearRightDriveReversed();

        public abstract boolean getkFrontLeftTurningReversed();

        public abstract boolean getkRearLeftTurningReversed();

        public abstract boolean getkFrontRightTurningReversed();

        public abstract boolean getkRearRightTurningReversed();

        public abstract double getkTrackWidth();

        public abstract double getkWheelBase();

        public abstract SwerveDriveKinematics getkDriveKinematics();

        public abstract boolean getkGyroReversed();

        public abstract double getksVolts();

        public abstract double getkvVoltSecondsPerMeter();

        public abstract double getkaVoltSecondsSquaredPerMeter();

        public abstract double getkMaxSpeedINPerSecond();

        public abstract double getkMaxTurnSpeedRadPerSecond();

    }

    public abstract class ModuleConstants {
        public abstract double getkMaxModuleAngularSpeedRadiansPerSecond();

        public abstract double getkMaxModuleAngularAccelerationRadiansPerSecondSquared();

        public abstract int getkTurningEncoderCPR();

        public abstract int getkDriveEncoderCPR();

        public abstract double getkWheelDiameterIN();

        public abstract double getkDriveEncoderDistancePerPulse();

        public abstract double getkDriveEncoderRpmToInps();

        public abstract double getkTurningEncoderDistancePerPulse();

        public abstract double getkPModuleTurningController();

        public abstract double getkPModuleDriveController();

        public abstract double getkFModuleDriveController();

    }

    public abstract class OIConstants {
        public abstract int getkDriverControllerPort();
        public abstract int getkOpControllerPort();
    }

    public abstract class AutoConstants {
        public abstract double getkMaxSpeedINPerSecond();

        public abstract double getkMaxAccelerationINPerSecondSquared();

        public abstract double getkMaxAngularSpeedRadiansPerSecond();

        public abstract double getkMaxAngularSpeedRadiansPerSecondSquared();

        public abstract double getkPXController();

        public abstract double getkPYController();

        public abstract double getkDXController();

        public abstract double getkDYController();

        public abstract double getkPThetaController();

        public abstract TrapezoidProfile.Constraints getkThetaControllerConstraints();

    }

    public abstract class ShooterConstants{
        public abstract double getkP();
        public abstract double getkI();
        public abstract double getkD();
        public abstract double getkF();

        public abstract double getShooterServoDegreeTurnLimit();
        public abstract double getShooterABSEncoderLimit();
    }
}
