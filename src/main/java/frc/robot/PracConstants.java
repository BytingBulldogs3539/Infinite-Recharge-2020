/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
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
        public int getIntakeMotorID()
        {
            return 73;
        }

        @Override
        public int getElevatorBaseMotorID()
        {
            // TODO: Read ID
            return 0;
        }

        @Override
        public int getElevatorLiftMotorID()
        {
            // TODO: Real ID
            return 0;
        }

        @Override
        public int getShooterMotorID()
        {
            // TODO: Real ID
            return 0;
        }

        @Override
        public int getClimbMotorAID()
        {
            // TODO: Real ID
            return 0;
        }

        @Override
        public int getClimbMotorBID()
        {
            // TODO: Real ID
            return 0;
        }

        @Override
        public int getBuddyClimbMotorID()
        {
            // TODO: Real ID
            return 0;
        }

        @Override
        public int getSpinnerMotorID()
        {
            // TODO: Real ID
            return 0;
        }

        @Override
        public Port getColorSensorPort()
        {
            return Port.kOnboard;
        }
    }

    public class DriveConstants extends Constants.DriveConstants {
        public int getkFrontLeftDriveMotorPort() {
            return 2;
        }

        public int getkRearLeftDriveMotorPort() {
            return 7;
        }

        public int getkFrontRightDriveMotorPort() {
            return 4;
        }

        public int getkRearRightDriveMotorPort() {
            return 5;
        }

        public int getkFrontLeftTurningMotorPort() {
            return 1;
        }

        public int getkRearLeftTurningMotorPort() {
            return 8;
        }

        public int getkFrontRightTurningMotorPort() {
            return 3;
        }

        public int getkRearRightTurningMotorPort() {
            return 6;
        }

        public int getkPigeonID() {
            return 25;
        }

        public int[] getkFrontLeftTurningEncoderPorts() {
            return new int[] {
                2,
                3
            };
        }

        public int[] getkRearLeftTurningEncoderPorts() {
            return new int[] {
                4,
                5
            };
        }

        public int[] getkFrontRightTurningEncoderPorts() {
            return new int[] {
                0,
                1
            };
        }

        public int[] getkRearRightTurningEncoderPorts() {
            return new int[] {
                6,
                7
            };
        }

        public boolean getkFrontLeftTurningEncoderReversed() {
            return false;
        }

        public boolean getkRearLeftTurningEncoderReversed() {
            return false;
        }

        public boolean getkFrontRightTurningEncoderReversed() {
            return false;
        }

        public boolean getkRearRightTurningEncoderReversed() {
            return false;
        }

        public boolean getkFrontLeftDriveEncoderReversed() {
            return false;
        }

        public boolean getkRearLeftDriveEncoderReversed() {
            return false;
        }

        public boolean getkFrontRightDriveEncoderReversed() {
            return false;
        }

        public boolean getkRearRightDriveEncoderReversed() {
            return false;
        }

        public boolean getkFrontLeftDriveReversed() {
            return true;
        }

        public boolean getkRearLeftDriveReversed() {
            return true;
        }

        public boolean getkFrontRightDriveReversed() {
            return true;
        }

        public boolean getkRearRightDriveReversed() {
            return true;
        }

        public boolean getkFrontLeftTurningReversed() {
            return false;
        }

        public boolean getkRearLeftTurningReversed() {
            return false;
        }

        public boolean getkFrontRightTurningReversed() {
            return false;
        }

        public boolean getkRearRightTurningReversed() {
            return false;
        }

        public double getkTrackWidth() {
            return 30;
        }

        // Distance between centers of right and left wheels on robot
        public double getkWheelBase() {
            return 30;
        }

        // Distance between front and back wheels on robot
        public SwerveDriveKinematics getkDriveKinematics() {
            return new SwerveDriveKinematics(new Translation2d(getkWheelBase() / 2, getkTrackWidth() / 2),
                new Translation2d(getkWheelBase() / 2, -getkTrackWidth() / 2),
                new Translation2d(-getkWheelBase() / 2, getkTrackWidth() / 2),
                new Translation2d(-getkWheelBase() / 2, -getkTrackWidth() / 2));
        }

        public boolean getkGyroReversed() {
            return false;
        }

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The RobotPy Characterization Toolsuite provides a convenient tool for
        // obtaining these
        // values for your robot.
        public double getksVolts() {
            return 1;
        }

        public double getkvVoltSecondsPerMeter() {
            return 0.8;
        }

        public double getkaVoltSecondsSquaredPerMeter() {
            return 0.15;
        }

        public double getkMaxSpeedINPerSecond() {
            return 160.44;
        }

        public double getkMaxTurnSpeedRadPerSecond() {
            return 10.6;

        }
    }

    public class ModuleConstants extends Constants.ModuleConstants {
        public double getkMaxModuleAngularSpeedRadiansPerSecond() {
            return (50 * Math.PI);
        }

        public double getkMaxModuleAngularAccelerationRadiansPerSecondSquared() {
            return (50 * Math.PI);
        }

        public int getkTurningEncoderCPR() {
            return 1024;
        }

        public int getkDriveEncoderCPR() {
            return 233;
        }

        public double getkWheelDiameterIN() {
            return 3.0;
        }

        public double getkDriveEncoderDistancePerPulse() {
            return ((getkWheelDiameterIN() * Math.PI) / (double) getkDriveEncoderCPR());
        }

        public double getkDriveEncoderRpmToInps() {
            return (157.44 / 5676.0);
        }

        public double getkTurningEncoderDistancePerPulse() {
            return
            // Assumes the encoders are on a 1:1 reduction with the module shaft.
            (2 * Math.PI) / (double) getkTurningEncoderCPR();
        }

        public double getkPModuleTurningController() {
            return .5;
        }

        public double getkPModuleDriveController() {
            return .0005;
        }

        public double getkFModuleDriveController() {
            return .006;

        }
    }
    public class OIConstants extends Constants.OIConstants {
        public int getkDriverControllerPort() {
            return 0;

        }
    }
    public class AutoConstants extends Constants.AutoConstants {
        public double getkMaxSpeedINPerSecond() {
            return 160.44;
        }

        public double getkMaxAccelerationINPerSecondSquared() {
            return 160.44*1.5;
        }

        public double getkMaxAngularSpeedRadiansPerSecond() {
            return 10.6;
        }

        public double getkMaxAngularSpeedRadiansPerSecondSquared() {
            return 10.6;
        }

        public double getkPXController() {
            return 1;
        }

        public double getkPYController() {
            return 1;
        }

        public double getkPThetaController() {
            return 1;
        }

        // Constraint for the motion profilied robot angle controller
        public TrapezoidProfile.Constraints getkThetaControllerConstraints() {
            return new TrapezoidProfile.Constraints(getkMaxAngularSpeedRadiansPerSecond(), getkMaxAngularSpeedRadiansPerSecondSquared());
        }
    }

    private RobotIDConstants robotIDConstants = new RobotIDConstants();

    private DriveConstants robotDriveConstants = new DriveConstants();

    private ModuleConstants robotModuleConstants = new ModuleConstants();

    private OIConstants robotOIConstants = new OIConstants();

    private AutoConstants robotAutoConstants = new AutoConstants();

    @Override
    public frc.robot.utilities.Constants.RobotIDConstants getRobotIDConstants() {
        return robotIDConstants;
    }

    @Override
    public frc.robot.utilities.Constants.DriveConstants getDriveConstants() {
        return robotDriveConstants;
    }

    @Override
    public frc.robot.utilities.Constants.ModuleConstants getModuleConstants() {
        return robotModuleConstants;
    }

    @Override
    public frc.robot.utilities.Constants.OIConstants getOIConstants() {
        return robotOIConstants;
    }

    @Override
    public frc.robot.utilities.Constants.AutoConstants getAutoConstants() {
        return robotAutoConstants;
    }

}