// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EverybotIntake extends SubsystemBase {

  private CANSparkMax intakeMotor;

  private Solenoid intakeEjector;

  private double motorSpeed = 0.0;

  /** Creates a new FlapperIntake. */
  public EverybotIntake() {
    intakeEjector = new Solenoid(2, PneumaticsModuleType.REVPH, 12);
 
    intakeMotor = new CANSparkMax(Constants.INTAKE_LEFT_MOTOR_ID, MotorType.kBrushless);


    intakeMotor.setIdleMode(IdleMode.kBrake);




    ShuffleboardTab tempTab = Shuffleboard.getTab("Everbot Intake");

      tempTab.addNumber("Intake Speed", () -> {return intakeMotor.get();});

  }

  public void setMotorSpeed(double runningSpeed) {
    motorSpeed = runningSpeed;
  }

  public void stopMotor() {
    motorSpeed = 0.0;
  }

  public void toggleIntakePosition() {
    intakeEjector.set(!intakeEjector.get());
  }
  public void lowerIntake() {
    intakeEjector.set(true);
  }
  public void liftIntake() {
    intakeEjector.set(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (motorSpeed != 0.0) {
      intakeMotor.set(motorSpeed);
    } else {

      intakeMotor.set(0);
    }
  }
}
