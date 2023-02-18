// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlapperIntake extends SubsystemBase {
  private WPI_CANCoder leftAngle;
  private WPI_CANCoder rightAngle;

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private Solenoid intakeEjector;

  private double motorSpeed = 0.0;

  private PIDController leftPID;
  private PIDController rightPID;

  Timer flapperTimer = new Timer();

  /** Creates a new FlapperIntake. */
  public FlapperIntake() {
    intakeEjector = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

    leftMotor = new CANSparkMax(24, MotorType.kBrushless);
    rightMotor = new CANSparkMax(23, MotorType.kBrushless);

    leftAngle = new WPI_CANCoder(22);
    rightAngle = new WPI_CANCoder(21);

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftAngle.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    rightAngle.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    leftAngle.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    rightAngle.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    rightAngle.configMagnetOffset(-1.934);
    leftAngle.configMagnetOffset(-70.752);
    leftAngle.setPosition(leftAngle.getAbsolutePosition());
    rightAngle.setPosition(rightAngle.getAbsolutePosition());

    leftPID = new PIDController(0.005, 0.025, 0.0);
    rightPID = new PIDController(0.005, 0.025, 0.0);

    ShuffleboardTab tempTab = Shuffleboard.getTab("Flapper Intake");
      tempTab.addNumber("Left Angle", () -> {return leftAngle.getPosition();});
      tempTab.addNumber("Right Angle", () -> {return rightAngle.getPosition();});
      tempTab.addNumber("Absolute Left Angle", () -> {return leftAngle.getAbsolutePosition();});
      tempTab.addNumber("Absolute Right Angle", () -> {return rightAngle.getAbsolutePosition();});
      tempTab.addNumber("Left Speed", () -> {return leftMotor.get();});
      tempTab.addNumber("Right Speed", () -> {return rightMotor.get();});
  }

  public void setMotorSpeed(double runningSpeed) {
    motorSpeed = runningSpeed;
  }

  public void stopMotor() {
    motorSpeed = 0.0;

    flapperTimer.reset();
  }

  public void toggleIntakePosition() {
    intakeEjector.set(!intakeEjector.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (motorSpeed != 0.0) {
      rightMotor.set(motorSpeed);
      leftMotor.set(-motorSpeed);
    } else {
      double leftSpeed = leftPID.calculate(leftAngle.getAbsolutePosition(), 0.0);
      double rightSpeed = rightPID.calculate(rightAngle.getAbsolutePosition(), 0.0);

      if (Math.abs(leftAngle.getAbsolutePosition()) < 2.0) {
        leftSpeed = 0.0;
      }

      if (Math.abs(rightAngle.getAbsolutePosition()) < 2.0) {
        rightSpeed = 0.0;
      }

      leftMotor.set(leftSpeed);      
      rightMotor.set(rightSpeed);
    }
  }
}
