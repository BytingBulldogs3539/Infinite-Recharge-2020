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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {

  // Shooter motor
  TalonFX shooterMotor = new TalonFX(RobotContainer.robotConstants.getRobotIDConstants().getShooterMotorID());
  

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    shooterMotor.setInverted(InvertType.InvertMotorOutput);
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
   * 
   * @param velocity
   * 
   */
  public void setVelocity(double velocity) {

  }

  /**
   * 
   * Returns the current velocity of the motor.
   * 
   */
  //     v Should not be void
  public void getVelocity() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
