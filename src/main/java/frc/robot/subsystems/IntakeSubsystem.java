/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utilities.ByteDoubleSolenoid;

public class IntakeSubsystem extends SubsystemBase
{

  // Intake motor
  CANSparkMax intakeMotor = new CANSparkMax(RobotContainer.robotConstants.getRobotIDConstants().getIntakeMotorID(),
      RobotContainer.robotConstants.getIntakeConstants().getIntakeMotorType());
  Compressor compressor = new Compressor(RobotContainer.robotConstants.getRobotIDConstants().getPCMID());
  ByteDoubleSolenoid intakSolenoid = new ByteDoubleSolenoid(
      RobotContainer.robotConstants.getRobotIDConstants().getPCMID(),
      RobotContainer.robotConstants.getRobotIDConstants().getIntakeSolinoidOn(),
      RobotContainer.robotConstants.getRobotIDConstants().getIntakeSolinoidOff(),
      RobotContainer.robotConstants.getIntakeConstants().getDefaultIntakeDirection());

  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem()
  {
    intakeMotor.setInverted(RobotContainer.robotConstants.getIntakeConstants().getIntakeMotorInverted());
    intakeMotor.setSmartCurrentLimit(35);
    // setDefaultCommand(new IntakeCommand(this));
    setCompressor(true);
  }

  /**
   * 
   * 
   * @param speed Motor power between -1 (reverse) and 1 (forwards/intake)
   * 
   */
  public void setPercentOutput(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * 
   * @param onOff false for compressor off true for compressor on.
   */
  public void setCompressor(boolean onOff) {
    if (onOff)
      compressor.start();
    else
      compressor.stop();
  }

  public void setIntakeSolinoid(boolean state) {
    if (state)
      intakSolenoid.set(Value.kForward);
    else
      intakSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
