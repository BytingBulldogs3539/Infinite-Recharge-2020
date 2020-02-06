/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ClimbSubsystem extends SubsystemBase {

  VictorSPX climbMotorL = new VictorSPX(RobotContainer.robotConstants.getRobotIDConstants().getClimbMotorLID());
  VictorSPX climbMotorB = new VictorSPX(RobotContainer.robotConstants.getRobotIDConstants().getClimbMotorRID());

  /**
   * Creates a new ClimbSubsystem.
   */
  public ClimbSubsystem() {
    if(RobotContainer.robotConstants.getClimbConstants().getClimbMotorBrake())
    {
      climbMotorL.setNeutralMode(NeutralMode.Brake);
      climbMotorB.setNeutralMode(NeutralMode.Brake);
    }
    else
    {
      climbMotorL.setNeutralMode(NeutralMode.Coast);
      climbMotorB.setNeutralMode(NeutralMode.Coast);
    }
    climbMotorL.setInverted(!RobotContainer.robotConstants.getClimbConstants().getClimbMotorInverted());
    climbMotorB.setInverted(RobotContainer.robotConstants.getClimbConstants().getClimbMotorInverted());
  }

  public void setPercentOutput(double power)
  {
    climbMotorL.set(ControlMode.PercentOutput, power);
    climbMotorB.set(ControlMode.PercentOutput, power);

  }
  public void setPercentOutputL(double power)
  {
    climbMotorL.set(ControlMode.PercentOutput, power);

  }
  public void setPercentOutputR(double power)
  {
    climbMotorB.set(ControlMode.PercentOutput, power);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
