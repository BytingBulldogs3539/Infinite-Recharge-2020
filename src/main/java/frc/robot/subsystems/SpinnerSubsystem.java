/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SpinnerSubsystem extends SubsystemBase {

  // Spinner motor
  TalonSRX spinnerMotor;
  ColorSensorV3 colorSensor = new ColorSensorV3(
      RobotContainer.robotConstants.getRobotIDConstants().getColorSensorPort());

  double rotations = 0;
  char startingColor = ' ';
  char lastColor = ' ';

  /**
   * Creates a new SpinnerSubsystem.
   */
  public SpinnerSubsystem() {
    spinnerMotor = RobotContainer.m_ClimbSubsystem.climbAdjuster;
  }

  Color color;
  double r, g, b;
  int proximity;

  public void resetRotations() {
    rotations = 0;
    startingColor = getCurrentColor();
    lastColor = startingColor;
  }

  public double getRotations() {
    return rotations;
  }

  public void updateRoations() {
    // Get the current color
    char currentColor = getCurrentColor();
    // if (currentColor != lastColor && currentColor == startingColor) {
    // rotations += 0.5;
    // }
    // TODO: figure out which way the control panel is spun to prevent issues (so
    // "rotations" will only increment one way)
    // Y -> B -> G -> R (counter-clockwise)
    if ((currentColor == 'Y' && lastColor == 'R') || (currentColor == 'B' && lastColor == 'Y')
        || (currentColor == 'G' && lastColor == 'B') || (currentColor == 'R' && lastColor == 'G')) {
      rotations += 0.125;
    }
    // R -> G -> B -> Y (clockwise)
    if ((currentColor == 'R' && lastColor == 'Y') || (currentColor == 'G' && lastColor == 'R')
        || (currentColor == 'B' && lastColor == 'G') || (currentColor == 'Y' && lastColor == 'B')) {
      rotations += 0.125;
    }
    // Update "lastColor" for the next iteration
    lastColor = currentColor;
  }

  public char getCurrentColor() {
    color = colorSensor.getColor();
    r = color.red;
    g = color.green;
    b = color.blue;
    proximity = colorSensor.getProximity();

    SmartDashboard.putNumber("colorSensor.getProximity()", colorSensor.getProximity());

    if (proximity >= 120) {
      if (r > g && r > b) {
        return 'R'; // red
      } else if (Math.abs(b - g) < 0.1) {
        return 'B'; // blue
      } else if (Math.abs(g - r) < 0.3 && b < 0.16) {
        return 'Y'; // yellow
      } else if (g > r && g > b) {
        return 'G'; // green
      } else {
        return 'N'; // NONE (no color)
      }
    } else {
      // Out of range shows the same as seeing no color
      return 'N';
    }
  }

  char col, newCol;

  public char getOffsetColor() {
    col = this.getCurrentColor();

    switch (col) {
    case 'R': // red
      newCol = 'B'; // change to blue
      break;
    case 'G': // green
      newCol = 'Y'; // change to yellow
      break;
    case 'B': // blue
      newCol = 'R'; // change to red
      break;
    case 'Y': // yellow
      newCol = 'G'; // change to green
      break;
    default:
      newCol = 'N';
      break;
    }

    return newCol;
  }

  public void setPercentOutput(double output) {
    spinnerMotor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void periodic() {
    /*
     * if(getCurrentColor() == 'B'){ // blue SmartDashboard.putString("Color",
     * "Blue"); }else if(getCurrentColor() == 'R'){ // red
     * SmartDashboard.putString("Color", "Red"); }else if(getCurrentColor() == 'Y'){
     * // yellow SmartDashboard.putString("Color", "Yellow"); }else
     * if(getCurrentColor() == 'G'){ // green SmartDashboard.putString("Color",
     * "Green"); }else if(getCurrentColor() == 'N'){ // NONE (no color)
     * SmartDashboard.putString("Color", "None"); }else{ //expected values are the 4
     * colors + 1 none condition. Anything else won't trip the if statments above.
     * SmartDashboard.putString("Color", "recived unexpected value"); }
     */
  }
}
