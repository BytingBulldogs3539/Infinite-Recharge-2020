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

public class IntakeSubsystem extends SubsystemBase {

  // Intake motor
  VictorSPX intakeMotor = new VictorSPX(RobotContainer.robotConstants.getRobotIDConstants().getIntakeMotorID());

  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {
  }

  /**
   * 
   * @param speed Motor power between -1 (reverse) and 1 (forwards/intake)
   * 
   */
  public void setPercentOutput(double speed) {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
