/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SpinnerSubsystem extends SubsystemBase
{

  // Spinner motor
  VictorSPX spinnerMotor = new VictorSPX(RobotContainer.robotConstants.getRobotIDConstants().getSpinnerMotorID());
  ColorSensorV3 colorSensor = new ColorSensorV3(
      RobotContainer.robotConstants.getRobotIDConstants().getColorSensorPort());

  /**
   * Creates a new SpinnerSubsystem.
   */
  public SpinnerSubsystem()
  {
  }

  Color color;
  double r, g, b;
  int proximity = colorSensor.getProximity();
  enum Col {
    BLUE,
    GREEN,
    RED,
    YELLOW,
    NONE
  }

  public Col getCurrentColor() {
    color = colorSensor.getColor();
    if(proximity >= 50){
    if (r > g && r > b) {
      return Col.RED;
    }
    else if (Math.abs(b - g) < 0.1)
    {
      return Col.BLUE;
    }
    else if (Math.abs(g - r) < 0.3 && b < 0.16)
    {
      return Col.YELLOW;
    }
    else if (g > r && g > b)
    {
      return Col.GREEN;
    }
    else
    {
      return Col.NONE;
    }
    }else{
      //Out of range shows the same as seeing no color
      return Col.NONE;
    }
  }


  Col col, newCol;

  public Col getOffsetColor() {
    col = this.getCurrentColor();

    switch (col)
    {
    case RED:
      newCol = Col.BLUE;
      break;
    case GREEN:
      newCol = Col.YELLOW;
      break;
    case BLUE:
      newCol = Col.RED;
      break;
    case YELLOW:
      newCol = Col.GREEN;
      break;
    case NONE:
      newCol = Col.NONE;
      break;
    }

    return newCol;
  }

  public void setPercentOutput(double output) {
    spinnerMotor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void periodic() {
   if(getCurrentColor() == Col.BLUE){
    SmartDashboard.putString("Color", "Blue");
   }else if(getCurrentColor() == Col.RED){
    SmartDashboard.putString("Color", "Red");
   }else if(getCurrentColor() == Col.YELLOW){
    SmartDashboard.putString("Color", "Yellow");
   }else if(getCurrentColor() == Col.GREEN){
    SmartDashboard.putString("Color", "Green");
   }else if(getCurrentColor() == Col.NONE){
    SmartDashboard.putString("Color", "None");
   }else{
     //expected values are the 4 colors + 1 none condition. Anything else won't trip the if statments above.
    SmartDashboard.putString("Color", "recived unexpected value");
   }
  }
}
