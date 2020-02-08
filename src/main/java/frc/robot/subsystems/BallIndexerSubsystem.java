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
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.BallIndexerCommand;

public class BallIndexerSubsystem extends SubsystemBase
{
  TalonSRX ballIndexerSrx = new TalonSRX(RobotContainer.robotConstants.getRobotIDConstants().getIndexMotorID());
  DigitalInput ballSensor = new DigitalInput(RobotContainer.robotConstants.getRobotIDConstants().getBallSensorPort());
  public PigeonIMU pigeon;

  /**
   * Creates a new BallIndexerSubsystem.
   */
  public BallIndexerSubsystem() {
    pigeon = new PigeonIMU(ballIndexerSrx);
    ballIndexerSrx.setInverted(RobotContainer.robotConstants.getBallIndexerConstants().getIndexMotorInverted());
    if(RobotContainer.robotConstants.getBallIndexerConstants().getIndexMotorBrake())
      ballIndexerSrx.setNeutralMode(NeutralMode.Brake);
    else
      ballIndexerSrx.setNeutralMode(NeutralMode.Coast);

      setDefaultCommand(new BallIndexerCommand(this));
  }

  public void setPercentOutput(double speed){
    ballIndexerSrx.set(ControlMode.PercentOutput, speed);
  }
  public boolean getBallSensorState()
  {
    return !ballSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
