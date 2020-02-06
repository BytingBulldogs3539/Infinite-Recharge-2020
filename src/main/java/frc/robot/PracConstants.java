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
            return 9;
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
            return 16;
        }

        @Override
        public int getIndexMotorID()
        {
            // TODO: Real ID
            return 11;
        }

        @Override
        public int getClimbMotorLID()
        {
            // TODO: Real ID
            return 10;
        }

        @Override
        public int getClimbMotorRID()
        {
            // TODO: Real ID
            return 15;
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
        public int getShooterServoID()
        {
            return 0;
        }

        @Override
        public int getShooterPotentiometerID()
        {
            return 0;
        }

        @Override
        public Port getColorSensorPort()
        {
            return Port.kOnboard;
        }
    }

    public class DriveConstants extends Constants.DriveConstants
    {
        public int getkFrontLeftDriveMotorPort()
        {
            return 7;
        }

        public int getkRearLeftDriveMotorPort()
        {
            return 2;
        }

        public int getkFrontRightDriveMotorPort()
        {
            return 5;
        }

        public int getkRearRightDriveMotorPort()
        {
            return 3;
        }

        public int getkFrontLeftTurningMotorPort()
        {
            return 8;
        }

        public int getkRearLeftTurningMotorPort()
        {
            return 1;
        }

        public int getkFrontRightTurningMotorPort()
        {
            return 6;
        }

        public int getkRearRightTurningMotorPort()
        {
            return 4;
        }

        public int getkPigeonID()
        {
            return 25;
        }

        public int getkFrontLeftTurningEncoderPorts()
        {
            return 30;
        }

        public int getkRearLeftTurningEncoderPorts()
        {
            return 32;
        }

        public int getkFrontRightTurningEncoderPorts()
        {
            return 33;
        }

        public int getkRearRightTurningEncoderPorts()
        {
            return 31;
        }

        public boolean getkFrontLeftTurningEncoderReversed()
        {
            return true;
        }

        public boolean getkRearLeftTurningEncoderReversed()
        {
            return true;
        }

        public boolean getkFrontRightTurningEncoderReversed()
        {
            return true;
        }

        public boolean getkRearRightTurningEncoderReversed()
        {
            return true;
        }

        public boolean getkFrontLeftDriveEncoderReversed()
        {
            return false;
        }

        public boolean getkRearLeftDriveEncoderReversed()
        {
            return false;
        }

        public boolean getkFrontRightDriveEncoderReversed()
        {
            return false;
        }

        public boolean getkRearRightDriveEncoderReversed()
        {
            return false;
        }

        public boolean getkFrontLeftDriveReversed()
        {
            return true;
        }

        public boolean getkRearLeftDriveReversed()
        {
            return true;
        }

        public boolean getkFrontRightDriveReversed()
        {
            return true;
        }

        public boolean getkRearRightDriveReversed()
        {
            return true;
        }

        public boolean getkFrontLeftTurningReversed()
        {
            return false;
        }

        public boolean getkRearLeftTurningReversed()
        {
            return false;
        }

        public boolean getkFrontRightTurningReversed()
        {
            return false;
        }

        public boolean getkRearRightTurningReversed()
        {
            return false;
        }

        public double getkTrackWidth()
        {
            return 30;
        }

        // Distance between centers of right and left wheels on robot
        public double getkWheelBase()
        {
            return 30;
        }

        // Distance between front and back wheels on robot
        public SwerveDriveKinematics getkDriveKinematics()
        {
            return new SwerveDriveKinematics(new Translation2d(getkWheelBase() / 2, getkTrackWidth() / 2),
                    new Translation2d(getkWheelBase() / 2, -getkTrackWidth() / 2),
                    new Translation2d(-getkWheelBase() / 2, getkTrackWidth() / 2),
                    new Translation2d(-getkWheelBase() / 2, -getkTrackWidth() / 2));
        }

        public boolean getkGyroReversed()
        {
            return false;
        }

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The RobotPy Characterization Toolsuite provides a convenient tool for
        // obtaining these
        // values for your robot.
        public double getksVolts()
        {
            return 1;
        }

        public double getkvVoltSecondsPerMeter()
        {
            return 0.8;
        }

        public double getkaVoltSecondsSquaredPerMeter()
        {
            return 0.15;
        }

        public double getkMaxSpeedINPerSecond()
        {
            return 176.52;
        }

        public double getkMaxTurnSpeedRadPerSecond()
        {
            return 10.6;

        }

        @Override
        public double getkFrontLeftTurningEncoderOffset()
        {
            return 162.949;
        }

        @Override
        public double getkRearLeftTurningEncoderOffset()
        {
            return 165.674;
        }

        @Override
        public double getkFrontRightTurningEncoderOffset()
        {
            return 95.801;
        }

        @Override
        public double getkRearRightTurningEncoderOffset()
        {
            return 24.609;
        }
    }

    public class ModuleConstants extends Constants.ModuleConstants
    {
        public double getkMaxModuleAngularSpeedRadiansPerSecond()
        {
            return (50 * Math.PI);
        }

        public double getkMaxModuleAngularAccelerationRadiansPerSecondSquared()
        {
            return (50 * Math.PI);
        }

        public int getkDriveEncoderCPR()
        {
            return 229;
        }

        public double getkWheelDiameterIN()
        {
            return 4.0;
        }

        public double getkDriveEncoderDistancePerPulse()
        {
            return ((getkWheelDiameterIN() * Math.PI) / (double) getkDriveEncoderCPR());
        }

        public double getkDriveEncoderRpmToInps()
        {
            return (157.44 / 5676.0);
        }

        public double getkPModuleTurningController()
        {
            return .25;
        }

        public double getkPModuleDriveController()
        {
            return .0001;
        }

        public double getkFModuleDriveController()
        {
            return .006;

        }
    }

    public class OIConstants extends Constants.OIConstants
    {
        public int getkDriverControllerPort()
        {
            return 0;
        }

        public int getkOpControllerPort()
        {
            return 1;
        }
    }

    public class AutoConstants extends Constants.AutoConstants
    {
        public double getkMaxSpeedINPerSecond()
        {
            return 160.44;
        }

        public double getkMaxAccelerationINPerSecondSquared()
        {
            return 20;
        }

        public double getkMaxAngularSpeedRadiansPerSecond()
        {
            return 10.6;
        }

        public double getkMaxAngularSpeedRadiansPerSecondSquared()
        {
            return 10.6;
        }

        public double getkPXController()
        {
            return .076;
        }

        public double getkPYController()
        {
            return .076;
        }

        public double getkDXController()
        {
            return .033;
        }

        public double getkDYController()
        {
            return .033;
        }

        public double getkPThetaController()
        {
            return 1.6;
        }

        // Constraint for the motion profilied robot angle controller
        public TrapezoidProfile.Constraints getkThetaControllerConstraints()
        {
            return new TrapezoidProfile.Constraints(getkMaxAngularSpeedRadiansPerSecond(),
                    getkMaxAngularSpeedRadiansPerSecondSquared());
        }
    }

    public class IntakeConstants extends Constants.IntakeConstants
    {

        @Override
        public MotorType getIntakeMotorType()
        {
            return MotorType.kBrushless;
        }

        @Override
        public boolean getIntakeMotorInverted()
        {
            return true;
        }

    }
    public class BallIndexerConstants extends Constants.BallIndexerConstants
    {

        @Override
        public boolean getIndexMotorInverted()
        {
            return true;
        }
        @Override
        public boolean getIndexMotorBrake()
        {
            return true;
        }
        
    }
    public class ClimbConstants extends Constants.ClimbConstants
    {

        @Override
        public boolean getClimbMotorInverted()
        {
            return true;
        }
        @Override
        public boolean getClimbMotorBrake()
        {
            return true;
        }
        
    }
    public class ShooterConstants extends Constants.ShooterConstants{

        @Override
        public double getkP() {
            // TODO tune values
            return 0;
        }

        @Override
        public double getkI() {
            // TODO tune values
            return 0;
        }

        @Override
        public double getkD() {
            // TODO tune values
            return 0;
        }

        @Override
        public double getkF() {
            // TODO tune values
            return 0;
        }

        @Override
        public double getPotOffset()
        {
            // TODO Auto-generated method stub
            return 0;
        }

        @Override
        public boolean getShooterMotorInverted()
        {
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

    @Override
    public frc.robot.utilities.Constants.ShooterConstants getShooterConstants() {
        return robotShooterConstants;
    }

    @Override
    public IntakeConstants getIntakeConstants()
    {
        return robotIntakeConstants;
    }

    @Override
    public BallIndexerConstants getBallIndexerConstants()
    {
        return robotBallIndexerConstants;
    }

    @Override
    public ClimbConstants getClimbConstants()
    {
        return robotClimberConstants;
    }

}