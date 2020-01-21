/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ElevatorSubsystem extends SubsystemBase {

  // TODO: Figure out if the two motors do what is desribed below
  // Motor for the base of the elevator
  VictorSPX elevatorBaseMotor = new VictorSPX(RobotContainer.robotConstants.getRobotIDConstants().getElevatorBaseMotorID());
  // Motor for the lift portion of the elevator
  VictorSPX elevatorLiftMotor = new VictorSPX(RobotContainer.robotConstants.getRobotIDConstants().getElevatorLiftMotorID());

  /**
   * Creates a new ElevatorSubsystem.
   */
  public ElevatorSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
