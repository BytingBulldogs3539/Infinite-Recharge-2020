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
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {

  AnalogPotentiometer potentiometer = new AnalogPotentiometer(RobotContainer.robotConstants.getRobotIDConstants().getShooterPotentiometerID(), 67.5, RobotContainer.robotConstants.getShooterConstants().getPotOffset());
  Servo servo = new Servo(RobotContainer.robotConstants.getRobotIDConstants().getShooterServoID());

  // Shooter motor
  TalonFX shooterMotor = new TalonFX(RobotContainer.robotConstants.getRobotIDConstants().getShooterMotorID());
  

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    shooterMotor.setInverted(RobotContainer.robotConstants.getShooterConstants().getShooterMotorInverted());
    shooterMotor.config_kP(0, RobotContainer.robotConstants.getShooterConstants().getkP());
    shooterMotor.config_kI(0, RobotContainer.robotConstants.getShooterConstants().getkI());
    shooterMotor.config_kD(0, RobotContainer.robotConstants.getShooterConstants().getkD());
    shooterMotor.config_kF(0, RobotContainer.robotConstants.getShooterConstants().getkF());

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
   * @param velocity the motor RPM (max motor rpm is about 5500)
   * 
   */
  public void setVelocity(double velocity) {
    shooterMotor.set(ControlMode.Velocity, velocity*2048.0/600.0);
  }

  /**
   * 
   * @return the current velocity of the motor in rpm.
   * 
   */
  public double getVelocity() {
    return shooterMotor.getSelectedSensorVelocity()/2048.0*600.0;
  }

  public double getHoodAngle() {
    return potentiometer.get();
  }
  
  public double setHoodAngle() {
    // TODO: change to wanted angle
    return SmartDashboard.getNumber("Servo Target", 1500);
  }

  public void setServoSpeed(double speed) {
    servo.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Curr Shooter Velocity", getVelocity());
  }
}
