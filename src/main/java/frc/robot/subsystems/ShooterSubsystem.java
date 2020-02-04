/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {

  AnalogInput analog = new AnalogInput(RobotContainer.robotConstants.getRobotIDConstants().getShooterPotentiometerID());
  Servo servo = new Servo(RobotContainer.robotConstants.getRobotIDConstants().getShooterServoID());

  // Shooter motor
  TalonFX shooterMotor = new TalonFX(RobotContainer.robotConstants.getRobotIDConstants().getShooterMotorID());
  

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    shooterMotor.setInverted(InvertType.InvertMotorOutput);
    shooterMotor.config_kP(0, RobotContainer.robotConstants.getShooterConstants().getkP());
    shooterMotor.config_kI(0, RobotContainer.robotConstants.getShooterConstants().getkI());
    shooterMotor.config_kD(0, RobotContainer.robotConstants.getShooterConstants().getkD());
    shooterMotor.config_kF(0, RobotContainer.robotConstants.getShooterConstants().getkF());
  }

  /**
   * 
   * @param speed
   * 
   */
  public void setPercentOutput(double speed) {
    shooterMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Sets the motor RPM of the shooter
   * @param velocity the motor RPM
   * 
   */
  public void setVelocity(double velocity) {
    shooterMotor.set(ControlMode.Velocity, velocity*2048.0/600.0);
  }

  /**
   * 
   * Returns the current velocity of the motor.
   * 
   */
  //     v Should not be void
  public double getVelocity() {
    return shooterMotor.getSelectedSensorVelocity()/2048.0*600.0;
  }

  public double getAnalog() {
    return analog.getValue();
  }
  
  public double getSetpoint() {
    // TODO: change to wanted angle
    return SmartDashboard.getNumber("Servo Target", 1500);
  }

  public void setServoSpeed(double speed) {
    servo.set(speed);
  }

  public double getDegrees() {
    return (getAnalog() / (RobotContainer.robotConstants.getShooterConstants().getShooterABSEncoderLimit() / RobotContainer.robotConstants.getShooterConstants().getShooterServoDegreeTurnLimit())) / 4.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Curr Shooter Velocity", getVelocity());
  }
}
