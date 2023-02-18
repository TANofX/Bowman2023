// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class ConveyerBelt extends SubsystemBase {
  private CANSparkMax conveyerBelt;
  /** Creates a new ConveyerBelt. */
  public ConveyerBelt(double conveyerSpeed) {
    conveyerBelt = new CANSparkMax(0,MotorType.kBrushless);
    double runningSpeed = 0;
     conveyerSpeed = runningSpeed;
    conveyerBelt.set(conveyerSpeed);
  }
;
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
public void Speed() {
  double slowConveyer = 0;
  double fastConveyer = 0;
}