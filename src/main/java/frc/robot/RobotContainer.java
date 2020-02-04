/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.utilities.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.net.NetworkInterface;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.VisionTrack;
import frc.robot.subsystems.BallIndexerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utilities.Constants;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive;
  public final ShooterSubsystem m_ShooterSubsystem;
  public final IntakeSubsystem m_IntakeSubsystem;
  public final BallIndexerSubsystem m_BallIndexerSubsystem;
  // The driver's controller
  public static XboxController m_driverController;
  public static XboxController m_opController;

  // The constants of the robot
  public static Constants robotConstants;

  // Define the possible robot MAC addresses so we can identify what robot we are
  // using.
  // TODO: Add Read MAC Addresses
  private static final byte[] COMPETITION_BOT_MAC_ADDRESS = new byte[] { 0x00, (byte) 0x80, 0x2f, 0x17, (byte) 0xe4,
      (byte) 0x4e };
  private static final byte[] PRACTICE_BOT_MAC_ADDRESS = new byte[] { 0x00, (byte) 0x80, 0x2f, 0x17, (byte) 0xe5,
      0x18 };


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    boolean competitionBot = false;
    boolean practiceBot = false;

    List<byte[]> macAddresses;
    try {
      macAddresses = getMacAddresses();
    } catch (IOException e) {
      // Don't crash, just log the stacktrace and continue without any mac addresses.
      DriverStation.reportError("Error Retrieving Mac Addresses", false);
      macAddresses = new ArrayList<>();
    }

    for (byte[] macAddress : macAddresses) {
      // First check if we are the competition bot
      if (Arrays.compare(COMPETITION_BOT_MAC_ADDRESS, macAddress) == 0) {
        competitionBot = true;
        break;
      }

      // Next check if we are the practice bot
      if (Arrays.compare(PRACTICE_BOT_MAC_ADDRESS, macAddress) == 0) {
        practiceBot = true;
        break;
      }
    }

    if (!competitionBot && !practiceBot) {
      String[] macAddressStrings = macAddresses.stream().map(RobotContainer::macToString).toArray(String[]::new);

      SmartDashboard.putStringArray("MAC Addresses", macAddressStrings);
      SmartDashboard.putString("Competition Bot MAC Address", macToString(COMPETITION_BOT_MAC_ADDRESS));
      SmartDashboard.putString("Practice Bot MAC Address", macToString(PRACTICE_BOT_MAC_ADDRESS));

      // If something goes terribly wrong we still want to use the competition bot
      // stuff in competition.
      competitionBot = true;
    }

    if (competitionBot) {
      robotConstants = new CompConstants();
    }

    m_robotDrive = new DriveSubsystem();
    m_ShooterSubsystem = new ShooterSubsystem();
    m_IntakeSubsystem = new IntakeSubsystem();
    m_BallIndexerSubsystem = new BallIndexerSubsystem();
    // Configure default commands
    m_robotDrive.setDefaultCommand(new DriveCommand(m_robotDrive));
    // Configure the button bindings
    configureButtonBindings();
   
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * calling passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController = new XboxController(robotConstants.getOIConstants().getkDriverControllerPort());
    m_opController = new XboxController(robotConstants.getOIConstants().getkOpControllerPort());
    JoystickButton button = new JoystickButton(m_driverController, 1);
    button.toggleWhenPressed(new VisionTrack(m_robotDrive));
    JoystickButton buttonb = new JoystickButton(m_driverController, 1);
    buttonb.whenPressed(new ShooterCommand(m_ShooterSubsystem, 5000, m_BallIndexerSubsystem));
  }

  /**
   * Gets the MAC addresses of all present network adapters.
   *
   * @return the MAC addresses of all network adapters.
   */
  private static List<byte[]> getMacAddresses() throws IOException {
    List<byte[]> macAddresses = new ArrayList<>();

    Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();

    NetworkInterface networkInterface;
    while (networkInterfaces.hasMoreElements()) {
      networkInterface = networkInterfaces.nextElement();

      byte[] address = networkInterface.getHardwareAddress();
      if (address == null) {
        continue;
      }

      macAddresses.add(address);
    }

    return macAddresses;
  }

  private static String macToString(byte[] address) {
    StringBuilder builder = new StringBuilder();
    for (int i = 0; i < address.length; i++) {
      if (i != 0) {
        builder.append(':');
      }
      builder.append(String.format("%02X", address[i]));
    }
    return builder.toString();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(robotConstants.getAutoConstants().getkMaxSpeedINPerSecond(),
    robotConstants.getAutoConstants().getkMaxAccelerationINPerSecondSquared())
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(robotConstants.getDriveConstants().getkDriveKinematics());
    //config.setReversed(true);
    
    //config.setReversed(true);
    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        //new Pose2d(36,36, Rotation2d.fromDegrees(90)), config);
        new Pose2d(0,-75, Rotation2d.fromDegrees(0)), config);
        
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        robotConstants.getDriveConstants().getkDriveKinematics(),

        // Position controllers
        new PIDController(robotConstants.getAutoConstants().getkPXController(), 0, robotConstants.getAutoConstants().getkDXController()), 
        new PIDController(robotConstants.getAutoConstants().getkPYController(), 0, robotConstants.getAutoConstants().getkDYController()),
        new ProfiledPIDController(robotConstants.getAutoConstants().getkPThetaController(), 0, 0, robotConstants.getAutoConstants().getkThetaControllerConstraints()),

        m_robotDrive::setModuleStates,

        m_robotDrive::getVisionSeeing,

        m_robotDrive::getVisionAngle,

        true,

        m_robotDrive

    );
    

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
