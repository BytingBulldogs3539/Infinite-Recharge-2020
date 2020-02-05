/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommand;

public class IntakeSubsystem extends SubsystemBase {

  // Intake motor
  CANSparkMax intakeMotor = new CANSparkMax(RobotContainer.robotConstants.getRobotIDConstants().getIntakeMotorID(), RobotContainer.robotConstants.getIntakeConstants().getIntakeMotorType());

  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {
    intakeMotor.setInverted(RobotContainer.robotConstants.getIntakeConstants().getIntakeMotorInverted());
    setDefaultCommand(new IntakeCommand(this));
  }

  /**
   * 
   * @param speed Motor power between -1 (reverse) and 1 (forwards/intake)
   * 
   */
  public void setPercentOutput(double speed) {
    //System.out.println(speed);
    intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
