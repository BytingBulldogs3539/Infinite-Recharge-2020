/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class BallIndexerSubsystem extends SubsystemBase {
  TalonSRX ballIndexerSrx = new TalonSRX(RobotContainer.robotConstants.getRobotIDConstants().getIndexMotorID());

  /**
   * Creates a new BallIndexerSubsystem.
   */
  public BallIndexerSubsystem() {
    ballIndexerSrx.setInverted(RobotContainer.robotConstants.getBallIndexerConstants().getIndexMotorInverted());
    if(RobotContainer.robotConstants.getBallIndexerConstants().getIndexMotorBrake())
      ballIndexerSrx.setNeutralMode(NeutralMode.Brake);
    else
      ballIndexerSrx.setNeutralMode(NeutralMode.Coast);
  }

  public void setPercentOutput(double speed){
    ballIndexerSrx.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
