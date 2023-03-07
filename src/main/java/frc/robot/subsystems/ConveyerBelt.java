// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class ConveyerBelt extends SubsystemBase {
  private CANSparkMax conveyerBelt;
  private double runningSpeed = 0;
  private DoubleSolenoid blockControl;
  
  /** Creates a new ConveyerBelt. */
  public ConveyerBelt() {
    conveyerBelt = new CANSparkMax(Constants.CONVEYER_SPARK_MAX_ID, MotorType.kBrushless);
    conveyerBelt.setOpenLoopRampRate(.25);

    blockControl = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 9, 8);
  }

  public void runConveyer(double speed) {
    runningSpeed = speed;
  }

  public void openConveyer() {
    blockControl.set(Value.kForward);
  }

  public void closeConveyer() {
    blockControl.set(Value.kReverse);
  }

  public DoubleSolenoid.Value getState() {
    return blockControl.get();
  }

  public void periodic() {
    // This method will be called once per scheduler run
    conveyerBelt.set(runningSpeed);
  }
}