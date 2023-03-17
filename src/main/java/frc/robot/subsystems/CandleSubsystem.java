// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class CandleSubsystem extends SubsystemBase {
  /** Creates a new CANdle. */
  private CANdle candle;
  public CandleSubsystem() {
    candle = new CANdle(Constants.CANDLE_ID);

    

  }
  public void cubeLight() {
    candle.setLEDs(48, 16, 107, 0, 0, 4);

  }
  public void coneLight() {
    candle.setLEDs(246, 190, 0, 0, 0, 4);
  }
  public void noLED() {
    candle.setLEDs(0, 0, 0, 0, 0, 0);
  }

  @Override
  public void periodic() {
    if (RobotContainer.m_arm.getArmPosition() == ArmPositions.UNKNOWN) {
      candle.setLEDs(255, 0, 0, 0, 4, 4);
    }
    else {
      candle.setLEDs(0, 0, 0, 0, 4, 4);
    }
    // This method will be called once per scheduler run
  }
}
