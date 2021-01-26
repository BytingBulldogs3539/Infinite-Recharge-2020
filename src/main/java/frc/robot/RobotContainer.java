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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.net.NetworkInterface;

import frc.robot.autons.SimpleThreeBall;
import frc.robot.autons.SixBallRonday;
import frc.robot.commands.BallIndexerManualCommand;
import frc.robot.commands.BuddyClimbCommand;
import frc.robot.commands.ClimbAdjustCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.CloseShooter;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ShooterChirpCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SpinnerCommand;
//import frc.robot.commands.ShooterChirpCommand;
import frc.robot.commands.VisionTrack;
import frc.robot.subsystems.BallIndexerSubsystem;
import frc.robot.subsystems.BuddyClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpinnerSubsystem;
import frc.robot.utilities.Constants;
import frc.robot.utilities.LogitechF310;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems
  public final ShooterSubsystem m_ShooterSubsystem;
  public final ClimbSubsystem m_ClimbSubsystem;
  public final IntakeSubsystem m_IntakeSubsystem;
  public final BallIndexerSubsystem m_BallIndexerSubsystem;
  public final BuddyClimbSubsystem m_BuddyClimbSubsystem;
  public final SpinnerSubsystem m_SpinnerSubsystem;
  public final DriveSubsystem m_robotDrive;

  // The driver's controller
  public static LogitechF310 m_driverController;
  public static LogitechF310 m_opController;

  // The constants of the robot
  public static Constants robotConstants;

  // Define the possible robot MAC addresses so we can identify what robot we are
  // using.
  // TODO: Add Read MAC Addresses
  private static final byte[] COMPETITION_BOT_MAC_ADDRESS = new byte[] { 0x00, (byte) 0x80, 0x2f, 0x28, (byte) 0x5B,
      (byte) 0x7A };
  private static final byte[] PRACTICE_BOT_MAC_ADDRESS = new byte[] { 0x00, (byte) 0x80, 0x2f, 0x17, (byte) 0xe4,
      (byte) 0x4e };

  public SendableChooser<Command> chooser = new SendableChooser<Command>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    boolean competitionBot = false;
    boolean practiceBot = false;

    List<byte[]> macAddresses;
    try
    {
      macAddresses = getMacAddresses();
    }
    catch (IOException e)
    {
      // Don't crash, just log the stacktrace and continue without any mac addresses.
      DriverStation.reportError("Error Retrieving Mac Addresses", false);
      macAddresses = new ArrayList<>();
    }

    for (byte[] macAddress : macAddresses)
    {
      // First check if we are the competition bot
      if (Arrays.compare(COMPETITION_BOT_MAC_ADDRESS, macAddress) == 0)
      {
        competitionBot = true;
        break;
      }

      // Next check if we are the practice bot
      if (Arrays.compare(PRACTICE_BOT_MAC_ADDRESS, macAddress) == 0)
      {
        practiceBot = true;
        break;
      }
    }

    if (!competitionBot && !practiceBot)
    {
      String[] macAddressStrings = macAddresses.stream().map(RobotContainer::macToString).toArray(String[]::new);

      SmartDashboard.putStringArray("MAC Addresses", macAddressStrings);
      SmartDashboard.putString("Competition Bot MAC Address", macToString(COMPETITION_BOT_MAC_ADDRESS));
      SmartDashboard.putString("Practice Bot MAC Address", macToString(PRACTICE_BOT_MAC_ADDRESS));

      // If something goes terribly wrong we still want to use the competition bot
      // stuff in competition.
      competitionBot = true;
    }

    if (competitionBot)
    {
      robotConstants = new CompConstants();
    }
    if (practiceBot)
    {
      robotConstants = new PracConstants();
    }

    m_ShooterSubsystem = new ShooterSubsystem();
    m_IntakeSubsystem = new IntakeSubsystem();
    m_BallIndexerSubsystem = new BallIndexerSubsystem();
    m_ClimbSubsystem = new ClimbSubsystem();
    m_BuddyClimbSubsystem = new BuddyClimbSubsystem();
    m_SpinnerSubsystem = new SpinnerSubsystem();
    m_robotDrive = new DriveSubsystem();
    // Configure the button bindings
    configureButtonBindings();
    putAuton();
  }
  public void putAuton()
  {
        chooser.addOption("Three Ball Trench", new SixBallRonday(m_robotDrive, m_IntakeSubsystem, m_ShooterSubsystem, m_BallIndexerSubsystem));
        chooser.addOption("AIM AND SHOOT ONLY", new SimpleThreeBall(m_robotDrive, m_IntakeSubsystem, m_ShooterSubsystem, m_BallIndexerSubsystem));
        SmartDashboard.putData("Auto Chooser", chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * calling passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController = new LogitechF310(robotConstants.getOIConstants().getKDriverControllerPort());
    m_opController = new LogitechF310(robotConstants.getOIConstants().getKOpControllerPort());


    m_driverController.buttonA.whenHeld(new VisionTrack(m_robotDrive));
    m_driverController.buttonSTART.whenPressed(new ResetGyro(m_robotDrive));
    m_driverController.buttonSELECT.whenPressed(new ResetEncoders(m_robotDrive));
    // m_driverController.buttonA.whenHeld(visionCommand);

    m_opController.buttonB.whenHeld(new ShooterCommand(m_ShooterSubsystem, m_IntakeSubsystem, 5100, m_BallIndexerSubsystem, 0.0));
    m_opController.buttonTR.whenHeld(new BallIndexerManualCommand(m_BallIndexerSubsystem, m_ShooterSubsystem, .4));
    m_opController.buttonTL.whenHeld(new ShooterCommand(m_ShooterSubsystem, m_IntakeSubsystem, 5100, m_BallIndexerSubsystem, 0.0, true));
    m_opController.buttonY.whenHeld(new BallIndexerManualCommand(m_BallIndexerSubsystem, m_ShooterSubsystem,-.4));
    m_opController.buttonBR.whenHeld(new ClimbAdjustCommand(m_ClimbSubsystem, 1));
    m_opController.buttonBL.whenHeld(new ClimbAdjustCommand(m_ClimbSubsystem, -1));
    m_opController.buttonX.whenHeld(new SpinnerCommand(m_SpinnerSubsystem));
    m_opController.buttonSELECT.whenHeld(new ShooterCommand(m_ShooterSubsystem, m_IntakeSubsystem, 0.0, m_BallIndexerSubsystem, 0.7));
    m_opController.buttonSTART.whenHeld(new CloseShooter(m_ShooterSubsystem, m_IntakeSubsystem, 4800, m_BallIndexerSubsystem, 0.0));

    // m_opController.buttonY.whenHeld(new
    // BallIndexerCommand(m_BallIndexerSubsystem, -1));
    // m_opController.buttonX.whenHeld(new ClimbCommand(m_ClimbSubsystem, 1, 1));
    m_opController.buttonPadUp.whenHeld(new ClimbCommand(m_ClimbSubsystem, 1, 1));
    m_opController.buttonPadDown.whenHeld(new IntakeCommand(m_IntakeSubsystem,true));
    m_opController.buttonPadLeft.whenHeld(new BuddyClimbCommand(m_BuddyClimbSubsystem, 1));

    SmartDashboard.putData("Play Bad Guy", new ShooterChirpCommand(m_ShooterSubsystem, "badguy.chrp", true, 3));
    SmartDashboard.putData("Climb Up Left", new ClimbCommand(m_ClimbSubsystem, -.2, 0));
    SmartDashboard.putData("Climb Up Right", new ClimbCommand(m_ClimbSubsystem, 0, -.2));
    SmartDashboard.putData("Climb Down Left", new ClimbCommand(m_ClimbSubsystem, .2, 0));
    SmartDashboard.putData("Climb Down Right", new ClimbCommand(m_ClimbSubsystem, 0, .2));
    SmartDashboard.putData("Climb Up Both", new ClimbCommand(m_ClimbSubsystem, -.2, -.2));
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
    while (networkInterfaces.hasMoreElements())
    {
      networkInterface = networkInterfaces.nextElement();

      byte[] address = networkInterface.getHardwareAddress();
      if (address == null)
      {
        continue;
      }

      macAddresses.add(address);
    }

    return macAddresses;
  }

  private static String macToString(byte[] address) {
    StringBuilder builder = new StringBuilder();
    for (int i = 0; i < address.length; i++)
    {
      if (i != 0)
      {
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
    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(robotConstants.getAutoConstants().getKMaxSpeedINPerSecond(),
    //     robotConstants.getAutoConstants().getKMaxAccelerationINPerSecondSquared())
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(robotConstants.getDriveConstants().getKDriveKinematics());

    // // config.setReversed(true);
    // // An example trajectory to follow. All units in inches.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(),
    //     // new Pose2d(36,36, Rotation2d.fromDegrees(90)), config);
    //     new Pose2d(0, -75, Rotation2d.fromDegrees(0)), config);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     robotConstants.getDriveConstants().getKDriveKinematics(),

    //     // Position controllers
    //     new PIDController(robotConstants.getAutoConstants().getKPXController(), 0,
    //         robotConstants.getAutoConstants().getKDXController()),
    //     new PIDController(robotConstants.getAutoConstants().getKPYController(), 0,
    //         robotConstants.getAutoConstants().getKDYController()),
    //     new ProfiledPIDController(robotConstants.getAutoConstants().getKPThetaController(), 0, 0,
    //         robotConstants.getAutoConstants().getKThetaControllerConstraints()),

    //     m_robotDrive::setModuleStates,

    //     m_robotDrive::getVisionSeeing,

    //     m_robotDrive::getVisionAngle,

    //     true,

    //     m_robotDrive

    // );

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    Command autonMode = null;
    // try {
    //   autonMode = ();
    //   System.out.println("HELLO");
    // } catch (InstantiationException e) {
    //   e.printStackTrace();
    // } catch (IllegalAccessException e) {
    //   e.printStackTrace();
    // }
    return (Command)chooser.getSelected();
  }
}
