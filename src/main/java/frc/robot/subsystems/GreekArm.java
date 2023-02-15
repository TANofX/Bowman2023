// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GreekArm extends SubsystemBase {
  /**
   *
   */
  private static final double TESTING_SPEED = .25;
  private static final double HERCULES_LENGTH = 38.586;
  private static final double ARTEMIS_LENGTH = 27.25;
  private static final double APOLLO_LENGTH = 32.0;

private DoubleSolenoid gripperControl;
private WPI_TalonFX shoulderControl;
private CANSparkMax elbowControl;

private WPI_CANCoder shoulderAngle;
private WPI_CANCoder elbowAngle;

private Translation3d currentHandPosition;
private Rotation2d currentShoulderAngle;
private Rotation2d currentElbowAngle;

private double targetShoulderSpeed;
private double targetElbowSpeed;

private PIDController shoulderSpeedController;
private PIDController elbowSpeedController;

  /** Creates a new GreekArm. */
  public GreekArm() {
    shoulderControl = new WPI_TalonFX(Constants.SHOULDER_MOTER_CAN_ID);
    elbowControl = new CANSparkMax(Constants.ELBOW_MOTOR_CAN_ID, MotorType.kBrushless);
    gripperControl = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.HAND_SOLENOID_FORWARD_ID, Constants.HAND_SOLENOID_REVERSE_ID);

    shoulderSpeedController = new PIDController(.075, 0, .001);
    elbowSpeedController = new PIDController(.075, 0, .001);

    shoulderAngle = new WPI_CANCoder(Constants.SHOULDER_ENCODER_CAN_ID);
    elbowAngle = new WPI_CANCoder(Constants.ELBOW_ENCODER_CAN_ID);

    elbowControl.setIdleMode(IdleMode.kBrake);
    shoulderControl.setNeutralMode(NeutralMode.Brake);
    elbowAngle.configMagnetOffset(275.0);
    elbowAngle.configSensorDirection(true);
    elbowAngle.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    elbowAngle.setPosition(elbowAngle.getAbsolutePosition());

    shoulderAngle.configMagnetOffset(237.0);
    shoulderAngle.configSensorDirection(true);
    shoulderAngle.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    shoulderAngle.setPosition(shoulderAngle.getAbsolutePosition());

    ShuffleboardTab armTab = Shuffleboard.getTab("Greek Arm");
    armTab.addNumber("Hand X", () -> {return currentHandPosition.getX();});
    armTab.addNumber("Hand Z", () -> {return currentHandPosition.getZ();});
    armTab.add("Gripper State", gripperControl);

    armTab.addNumber("Elbow Angle", () -> { return currentElbowAngle.getDegrees();});
    armTab.addNumber("Elbow Absolute Angle", () -> { return elbowAngle.getAbsolutePosition();});

    armTab.addNumber("Shoulder Angle", () -> {return currentShoulderAngle.getDegrees();});
    armTab.addNumber("Shoulder Absolute Angle", () -> {return shoulderAngle.getAbsolutePosition();});
  }

  public static Translation3d calculateKinematics(double shoulderThetaDegrees, double elbowThetaDegrees) {
    double sTR = shoulderThetaDegrees * Math.PI / 180.0;
    double eTR = elbowThetaDegrees * Math.PI / 180.0;
    double tTR = sTR + eTR;

    double x = APOLLO_LENGTH * Math.cos(tTR) + ARTEMIS_LENGTH * Math.cos(sTR);
    double z = HERCULES_LENGTH + APOLLO_LENGTH * Math.sin(tTR) + ARTEMIS_LENGTH * Math.sin(sTR);

    return new Translation3d(x, 0.0, z);
  }

  public void setJointSpeeds(double shoulderSpeed, double elbowSpeed) {
    targetShoulderSpeed = shoulderSpeed;
    targetElbowSpeed = elbowSpeed;


  }

  public static List<Rotation2d> calculateInverseKinematics(double x, double z) {
    ArrayList<Rotation2d> returnList = new ArrayList<Rotation2d>(2);
    double zPrime = z - HERCULES_LENGTH;

    double elbow = Math.acos((Math.pow(x, 2.0) + Math.pow(zPrime, 2.0) - (Math.pow(ARTEMIS_LENGTH, 2.0) + Math.pow(APOLLO_LENGTH, 2.0))) / (2.0 * APOLLO_LENGTH * ARTEMIS_LENGTH));
    double shoulder = Math.atan2(zPrime, x) - Math.atan2(APOLLO_LENGTH * Math.sin(elbow), ARTEMIS_LENGTH + APOLLO_LENGTH * Math.cos(elbow));

    returnList.add(new Rotation2d(shoulder));
    returnList.add(new Rotation2d(elbow));

    elbow = -1 * Math.acos((Math.pow(x, 2.0) + Math.pow(zPrime, 2.0) - (Math.pow(ARTEMIS_LENGTH, 2.0) + Math.pow(APOLLO_LENGTH, 2.0))) / (2.0 * APOLLO_LENGTH * ARTEMIS_LENGTH));
    shoulder = Math.atan2(zPrime, x) - Math.atan2(APOLLO_LENGTH * Math.sin(elbow), ARTEMIS_LENGTH + APOLLO_LENGTH * Math.cos(elbow));

    returnList.add(new Rotation2d(shoulder));
    returnList.add(new Rotation2d(elbow));

    return returnList;
  }
  public void toggleGripper() {
    if (gripperControl.get().equals(Value.kReverse)) {
      gripperControl.set(Value.kForward);
    }
    else {
      gripperControl.set(Value.kReverse);
  }
}

public void moveshoulderup (){
  targetShoulderSpeed = TESTING_SPEED;
}
public void moveshoulderdown (){
 targetShoulderSpeed = -TESTING_SPEED;
}
public void stopshoulder (){
  shoulderControl.stopMotor();
  targetShoulderSpeed = 0;
}



public void moveelbowup (){
  targetElbowSpeed = TESTING_SPEED;
}
public void elbowdown (){
  targetElbowSpeed = -TESTING_SPEED;

}
public void stopelbow (){
  elbowControl.stopMotor();
  targetElbowSpeed = 0;

}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateState();
    double shoulderMotor = shoulderSpeedController.calculate(shoulderAngle.getVelocity(), targetShoulderSpeed);
    double elbowMotor = elbowSpeedController.calculate(elbowAngle.getVelocity(), targetElbowSpeed);

    shoulderControl.set(ControlMode.PercentOutput, targetShoulderSpeed);
    elbowControl.set(targetElbowSpeed);
    SmartDashboard.putNumber("shoulderDemand", shoulderMotor);
    SmartDashboard.putNumber("ElbowDemand", elbowMotor);



   // shoulderControl.set(ControlMode.PercentOutput, shoulderMotor);
   // elbowControl.set(elbowMotor);

  }
  private void updateState() {
    currentShoulderAngle = Rotation2d.fromDegrees(normalizeAngle(shoulderAngle.getPosition()));
    currentElbowAngle = Rotation2d.fromDegrees(normalizeAngle(elbowAngle.getPosition()));
    currentHandPosition = calculateKinematics(currentShoulderAngle.getDegrees(), currentElbowAngle.getDegrees());
  }

  private double normalizeAngle(double angleInDegrees) {
    if (angleInDegrees > 180.0) {
      return angleInDegrees - 360.0;
    }

    if (angleInDegrees < -180.0) {
      return angleInDegrees + 360.0;
    }

    return angleInDegrees;
  }
}
