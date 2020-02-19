/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import com.ctre.phoenix.music.Orchestra;

public class ShooterSubsystem extends SubsystemBase
{

  AnalogPotentiometer potentiometer = new AnalogPotentiometer(
      RobotContainer.robotConstants.getRobotIDConstants().getShooterPotentiometerID(), 67.5,
      RobotContainer.robotConstants.getShooterConstants().getPotOffset());
  Servo servo = new Servo(RobotContainer.robotConstants.getRobotIDConstants().getShooterServoID());

  // Shooter motor
  TalonFX shooterMotor = new TalonFX(RobotContainer.robotConstants.getRobotIDConstants().getShooterMotorID());

  // orchestra
  Orchestra _orchestra;
  ArrayList<TalonFX> _instruments = new ArrayList<TalonFX>();

  String currentSong = "";

  double hoodAngle = -1;

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem()
  {
    shooterMotor.setInverted(RobotContainer.robotConstants.getShooterConstants().getShooterMotorInverted());
    shooterMotor.config_kP(0, RobotContainer.robotConstants.getShooterConstants().getKP());
    shooterMotor.config_kI(0, RobotContainer.robotConstants.getShooterConstants().getKI());
    shooterMotor.config_kD(0, RobotContainer.robotConstants.getShooterConstants().getKD());
    shooterMotor.config_kF(0, RobotContainer.robotConstants.getShooterConstants().getKF());
    _orchestra = new Orchestra(_instruments);
    _orchestra.addInstrument(shooterMotor);

  }

  /**
   * 
   * @param speed a value between 1 (full forward) and -1 (full reverse)
   * 
   */
  public void setPercentOutput(double speed) {
    shooterMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Sets the motor RPM of the shooter
   * 
   * @param velocity the motor RPM (max motor rpm is about 5700)
   * 
   */
  public void setVelocity(double velocity) {
    shooterMotor.set(ControlMode.Velocity, velocity * 2048.0 / 600.0);
  }

  /**
   * 
   * @return the current velocity of the motor in rpm.
   * 
   */
  public double getVelocity() { return shooterMotor.getSelectedSensorVelocity() / 2048.0 * 600.0; }

  /**
   * 
   * @return returns the current Hood Angle in Degrees.
   */
  public double getHoodAngle() { return potentiometer.get(); }

  /**
   * 
   * @param angle the angle you want the hood to be at. (Zero will shoot the ball
   *          high and 60 will shoot it low almost into the floor.) a value of
   *          less than 0 or greater than 60 will disable pid.
   */
  public void setHoodAngle(double angle) { this.hoodAngle = angle; }

  public void setServoSpeed(double speed) {
    servo.set(speed);
  }

  public void loadMusic(String path) {
    if (!path.equals(this.currentSong))
      _orchestra.loadMusic(path);

  }

  /**
   * Plays or pauses the current music file in the Talon fx
   * 
   * @param play_pause True for play, False for pause
   * 
   */
  public void PlayPauseChirp(boolean play_pause) {
    if (play_pause)
    {
      _orchestra.play();

    }
    else
    {
      _orchestra.pause();
    }
  }

  public boolean isPlaying() { return _orchestra.isPlaying(); }

  public void StopChirp() {
    _orchestra.stop();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Curr Shooter Velocity", getVelocity());
  }
}
