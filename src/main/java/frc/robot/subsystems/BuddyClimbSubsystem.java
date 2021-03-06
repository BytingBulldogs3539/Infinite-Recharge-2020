/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class BuddyClimbSubsystem extends SubsystemBase
{

  // Buddy climb motor
  VictorSPX buddyClimbMotor = new VictorSPX(RobotContainer.robotConstants.getRobotIDConstants().getBuddyClimbMotorID());

  /**
   * Creates a new BuddyClimbSubsystem.
   */
  public BuddyClimbSubsystem()
  {
    buddyClimbMotor.setInverted(RobotContainer.robotConstants.getClimbConstants().getBuddyClimbMotorInverted());
  }

  public void setPercentOutput(double power) {
    buddyClimbMotor.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
