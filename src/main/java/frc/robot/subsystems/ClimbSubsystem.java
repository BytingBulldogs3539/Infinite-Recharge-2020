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
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ClimbSubsystem extends SubsystemBase
{

  VictorSPX climbMotorL = new VictorSPX(RobotContainer.robotConstants.getRobotIDConstants().getClimbMotorLID());
  VictorSPX climbMotorR = new VictorSPX(RobotContainer.robotConstants.getRobotIDConstants().getClimbMotorRID());
  public static TalonSRX climbAdjuster = new TalonSRX(RobotContainer.robotConstants.getRobotIDConstants().getAdjusterID());


  public static PigeonIMU pigeon;
  /**
   * Creates a new ClimbSubsystem.
   */
  public ClimbSubsystem()
  {
    pigeon = new PigeonIMU(climbAdjuster);
    if (RobotContainer.robotConstants.getClimbConstants().getClimbMotorBrake())
    {
      climbMotorL.setNeutralMode(NeutralMode.Brake);
      climbMotorR.setNeutralMode(NeutralMode.Brake);
    }
    else
    {
      climbMotorL.setNeutralMode(NeutralMode.Coast);
      climbMotorR.setNeutralMode(NeutralMode.Coast);
    }

    climbMotorL.setInverted(RobotContainer.robotConstants.getClimbConstants().getClimbMotorInverted());
    climbMotorR.setInverted(RobotContainer.robotConstants.getClimbConstants().getClimbMotorInverted());
    climbAdjuster.setNeutralMode(NeutralMode.Brake);
  }

  public void setPercentOutput(double power) {
    climbMotorL.set(ControlMode.PercentOutput, power);
    climbMotorR.set(ControlMode.PercentOutput, power);

  }

  public void setPercentOutputL(double power) {
    climbMotorL.set(ControlMode.PercentOutput, power);

  }

  public void setPercentOutputR(double power) {
    climbMotorR.set(ControlMode.PercentOutput, power);

  }
  public void setAdjusterPercentOutput(double power)
  {
    climbAdjuster.set(ControlMode.PercentOutput, -power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
