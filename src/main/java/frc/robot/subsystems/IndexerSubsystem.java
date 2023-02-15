// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class IndexerSubsystem extends PIDSubsystem {
  /** Creates a new IndexerSubsystem. */
  private CANSparkMax beltMotor;

  public IndexerSubsystem() {

    super(
        // The PIDController used by the subsystem
        new PIDController(0.01, 0, 0.001));
        beltMotor = new CANSparkMax(Constants.INDEXER_BELT_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
