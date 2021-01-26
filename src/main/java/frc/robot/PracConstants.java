/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.utilities.Constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class PracConstants extends Constants
{

    // TODO: Figure out real IDs
    public class RobotIDConstants extends Constants.RobotIDConstants
    {
        @Override
        public int getIntakeMotorID() { return 9; }

        @Override
        public int getShooterMotorID() { return 16; }

        @Override
        public int getIndexMotorID() { return 11; }

        @Override
        public int getClimbMotorLID() { return 10; }

        @Override
        public int getClimbMotorRID() { return 15; }

        @Override
        public int getAdjusterID() {
            // TODO Enter Real ID
            return 12;
        }

        @Override
        public int getBuddyClimbMotorID() {
            // TODO: Enter Real ID
            return 13;
        }

        @Override
        public int getSpinnerMotorID() { return 14; }

        @Override
        public int getShooterServoLID() {
            return 1;
        }
        @Override
        public int getShooterServoRID() {
            return 0;
        }

        @Override
        public int getShooterPotentiometerID() {
            // TODO: Enter Real ID
            return 0;
        }

        @Override
        public Port getColorSensorPort() { return Port.kOnboard; }

        @Override
        public int getPCMID() { return 20; }

        @Override
        public int getIntakeSolinoidOn() { return 1; }

        @Override
        public int getIntakeSolinoidOff() { return 0; }

        @Override
        public int getBallSensorPort() { return 0; }

    }

    public class DriveConstants extends Constants.DriveConstants
    {
        @Override
        public int getKFrontLeftDriveMotorPort() { return 7; }

        public int getKRearLeftDriveMotorPort() { return 2; }

        public int getKFrontRightDriveMotorPort() { return 5; }

        public int getKRearRightDriveMotorPort() { return 3; }

        public int getKFrontLeftTurningMotorPort() { return 8; }

        public int getKRearLeftTurningMotorPort() { return 1; }

        public int getKFrontRightTurningMotorPort() { return 6; }

        public int getKRearRightTurningMotorPort() { return 4; }

        public int getKPigeonID() { return 25; }

        public int getKFrontLeftTurningEncoderPorts() { return 30; }

        public int getKRearLeftTurningEncoderPorts() { return 32; }

        public int getKFrontRightTurningEncoderPorts() { return 33; }

        public int getKRearRightTurningEncoderPorts() { return 31; }

        public boolean getKFrontLeftTurningEncoderReversed() { return false; }

        public boolean getKRearLeftTurningEncoderReversed() { return false; }

        public boolean getKFrontRightTurningEncoderReversed() { return false; }

        public boolean getKRearRightTurningEncoderReversed() { return false; }

        public boolean getKFrontLeftDriveEncoderReversed() { return false; }

        public boolean getKRearLeftDriveEncoderReversed() { return false; }

        public boolean getKFrontRightDriveEncoderReversed() { return false; }

        public boolean getKRearRightDriveEncoderReversed() { return false; }

        public boolean getKFrontLeftDriveReversed() { return true; }

        public boolean getKRearLeftDriveReversed() { return true; }

        public boolean getKFrontRightDriveReversed() { return true; }

        public boolean getKRearRightDriveReversed() { return true; }

        public boolean getKFrontLeftTurningReversed() { return true; }

        public boolean getKRearLeftTurningReversed() { return true; }

        public boolean getKFrontRightTurningReversed() { return true; }

        public boolean getKRearRightTurningReversed() { return true; }

        public double getKTrackWidth() { return 30; }

        // Distance between centers of right and left wheels on robot
        public double getKWheelBase() { return 30; }

        // Distance between front and back wheels on robot
        public SwerveDriveKinematics getKDriveKinematics() {
            return new SwerveDriveKinematics(new Translation2d(getKWheelBase() / 2, getKTrackWidth() / 2),
                    new Translation2d(getKWheelBase() / 2, -getKTrackWidth() / 2),
                    new Translation2d(-getKWheelBase() / 2, getKTrackWidth() / 2),
                    new Translation2d(-getKWheelBase() / 2, -getKTrackWidth() / 2));
        }

        public boolean getKGyroReversed() { return true; }

        public double getKMaxSpeedINPerSecond() { return 176.52; }

        public double getKMaxTurnSpeedRadPerSecond() {
            return 10.6;

        }

        @Override
        public double getKFrontLeftTurningEncoderOffset() { return 14.502; }

        @Override
        public double getKRearLeftTurningEncoderOffset() { return -9.404297; }

        @Override
        public double getKFrontRightTurningEncoderOffset() { return -53.174; }

        @Override
        public double getKRearRightTurningEncoderOffset() { return -124.365; }

        public TrajectoryConfig getTrajectoryConfig() {
            return new TrajectoryConfig(RobotContainer.robotConstants.getAutoConstants().getKMaxSpeedINPerSecond(),
                    RobotContainer.robotConstants.getAutoConstants().getKMaxAccelerationINPerSecondSquared())
                            // Add kinematics to ensure max speed is actually obeyed
                            .setKinematics(RobotContainer.robotConstants.getDriveConstants().getKDriveKinematics());
        }
    }

    public class ModuleConstants extends Constants.ModuleConstants
    {
        public double getKMaxModuleAngularSpeedRadiansPerSecond() { return (50 * Math.PI); }

        public double getKMaxModuleAngularAccelerationRadiansPerSecondSquared() { return (50 * Math.PI); }

        public int getKDriveEncoderCPR() { return 240; }

        public double getKWheelDiameterIN() { return 4.0; }

        public double getKDriveEncoderDistancePerPulse() {
            return ((getKWheelDiameterIN() * Math.PI) / (double) getKDriveEncoderCPR());
        }

        public double getKDriveEncoderRpmToInps() { return (217.92 / 5676.0); }

        public double getKPModuleTurningController() { return .4; }

        public double getKPModuleDriveController() { return .0001; }

        public double getKFModuleDriveController() {
            return .006;

        }
    }

    public class OIConstants extends Constants.OIConstants
    {
        public int getKDriverControllerPort() { return 0; }

        public int getKOpControllerPort() { return 1; }
    }

    public class AutoConstants extends Constants.AutoConstants
    {
        public double getKMaxSpeedINPerSecond() { return 217.92; }

        public double getKMaxAccelerationINPerSecondSquared() { return 20; }

        public double getKMaxAngularSpeedRadiansPerSecond() { return 10.6; }

        public double getKMaxAngularSpeedRadiansPerSecondSquared() { return 10.6; }

        public double getKPXController() { return .076; }

        public double getKPYController() { return .076; }

        public double getKDXController() { return .033; }

        public double getKDYController() { return .033; }

        public double getKPThetaController() { return 1.6; }

        // Constraint for the motion profilied robot angle controller
        public TrapezoidProfile.Constraints getKThetaControllerConstraints() {
            return new TrapezoidProfile.Constraints(getKMaxAngularSpeedRadiansPerSecond(),
                    getKMaxAngularSpeedRadiansPerSecondSquared());
        }
    }

    public class IntakeConstants extends Constants.IntakeConstants
    {

        @Override
        public MotorType getIntakeMotorType() { return MotorType.kBrushless; }

        @Override
        public boolean getIntakeMotorInverted() { return false; }

        @Override
        public boolean getDefaultIntakeDirection() { return false; }

        @Override
        public int intakeReverseDelay() { return 5; }

    }

    public class BallIndexerConstants extends Constants.BallIndexerConstants
    {

        @Override
        public boolean getIndexMotorInverted() { return true; }

        @Override
        public boolean getIndexMotorBrake() { return true; }

    }

    public class ClimbConstants extends Constants.ClimbConstants
    {

        @Override
        public boolean getClimbMotorInverted() { return true; }

        @Override
        public boolean getClimbMotorBrake() { return true; }

        @Override
        public boolean getClimbAdjusterMotorInverted() { return true; }

        @Override
        public boolean getClimbAdjusterMotorBrake() { return true; }

    }

    public class ShooterConstants extends Constants.ShooterConstants
    {

        @Override
        public double getKP() {
            return 0.1;
        }

        @Override
        public double getKI() {
            return 0;
        }

        @Override
        public double getKD() {
            return 0;
        }

        @Override
        public double getKF() {
            return 0.054;
        }

        @Override
        public double getPotOffset() {
            return 0;
        }

        @Override
        public boolean invertHoodAngle() { return false; }

        @Override
        public double getHoodOffset(){
            return 74.3;
        }

        @Override
        public boolean getShooterMotorInverted() { return false; }

        @Override
        public boolean getSpinnerReversed() {
            return false;
        }

    }

    private RobotIDConstants robotIDConstants = new RobotIDConstants();

    private DriveConstants robotDriveConstants = new DriveConstants();

    private ModuleConstants robotModuleConstants = new ModuleConstants();

    private OIConstants robotOIConstants = new OIConstants();

    private AutoConstants robotAutoConstants = new AutoConstants();

    private IntakeConstants robotIntakeConstants = new IntakeConstants();

    private BallIndexerConstants robotBallIndexerConstants = new BallIndexerConstants();

    private ClimbConstants robotClimberConstants = new ClimbConstants();

    private ShooterConstants robotShooterConstants = new ShooterConstants();

    @Override
    public frc.robot.utilities.Constants.RobotIDConstants getRobotIDConstants() { return robotIDConstants; }

    @Override
    public frc.robot.utilities.Constants.DriveConstants getDriveConstants() { return robotDriveConstants; }

    @Override
    public frc.robot.utilities.Constants.ModuleConstants getModuleConstants() { return robotModuleConstants; }

    @Override
    public frc.robot.utilities.Constants.OIConstants getOIConstants() { return robotOIConstants; }

    @Override
    public frc.robot.utilities.Constants.AutoConstants getAutoConstants() { return robotAutoConstants; }

    @Override
    public frc.robot.utilities.Constants.ShooterConstants getShooterConstants() { return robotShooterConstants; }

    @Override
    public IntakeConstants getIntakeConstants() { return robotIntakeConstants; }

    @Override
    public BallIndexerConstants getBallIndexerConstants() { return robotBallIndexerConstants; }

    @Override
    public ClimbConstants getClimbConstants() { return robotClimberConstants; }

}