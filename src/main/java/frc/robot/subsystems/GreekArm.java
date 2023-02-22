// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
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

  private class JointSpeeds {
    private Rotation2d eSpeed;
    private Rotation2d sSpeed;

    public JointSpeeds() {
      eSpeed = Rotation2d.fromRadians(0.0);
      sSpeed = Rotation2d.fromRadians(0.0);
    };

    public JointSpeeds(double shoulderRadiansSpeed, double elbowRadiansSpeed) {
      eSpeed = Rotation2d.fromRadians(elbowRadiansSpeed);
      sSpeed = Rotation2d.fromRadians(shoulderRadiansSpeed);
    }

    public JointSpeeds(Rotation2d shoulderSpeed, Rotation2d elbowSpeed) {
      eSpeed = elbowSpeed;
      sSpeed = shoulderSpeed;
    }

    public Rotation2d getShoulderSpeed() {
      return sSpeed;
    }

    public Rotation2d getElbowSpeed() {
      return eSpeed;
    }
  }
  /**
   *
   */
  
private DoubleSolenoid gripperControl;
private WPI_TalonFX shoulderControl;
private CANSparkMax elbowControl;

private WPI_CANCoder shoulderAngle;
private WPI_CANCoder elbowAngle;

private Translation3d currentHandPosition;
private Rotation2d currentShoulderAngle;
private Rotation2d currentElbowAngle;

private Rotation2d targetShoulderSpeed  = new Rotation2d();
private Rotation2d targetElbowSpeed = new Rotation2d();

private PIDController shoulderSpeedController;
private PIDController elbowSpeedController;

  /** Creates a new GreekArm. */
  public GreekArm() {
    shoulderControl = new WPI_TalonFX(Constants.SHOULDER_MOTER_CAN_ID);
    elbowControl = new CANSparkMax(Constants.ELBOW_MOTOR_CAN_ID, MotorType.kBrushless);
    gripperControl = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.HAND_SOLENOID_FORWARD_ID, Constants.HAND_SOLENOID_REVERSE_ID);

    shoulderSpeedController = new PIDController(.002, 0, .00);
    elbowSpeedController = new PIDController(.002, 0, .00);

    shoulderAngle = new WPI_CANCoder(Constants.SHOULDER_ENCODER_CAN_ID);
    elbowAngle = new WPI_CANCoder(Constants.ELBOW_ENCODER_CAN_ID);

    elbowControl.setIdleMode(IdleMode.kBrake);
    shoulderControl.setNeutralMode(NeutralMode.Brake);
    elbowAngle.configMagnetOffset(274.219 + 4.8);
    elbowAngle.configSensorDirection(true);
    elbowAngle.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    elbowAngle.setPosition(elbowAngle.getAbsolutePosition());
    elbowAngle.getVelocity();

    shoulderAngle.configMagnetOffset(303.553);
    shoulderAngle.configSensorDirection(true);
    shoulderAngle.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    shoulderAngle.setPosition(shoulderAngle.getAbsolutePosition());
    shoulderAngle.getVelocity();
    shoulderAngle.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    ShuffleboardTab armTab = Shuffleboard.getTab("Greek Arm");
    armTab.addNumber("shoulderVelocity", () -> {return shoulderAngle.getVelocity();});
    armTab.addNumber("elbowVelocity", () -> {return elbowAngle.getVelocity();});

    armTab.addNumber("Hand X", () -> {return currentHandPosition.getX();});
    armTab.addNumber("Hand Z", () -> {return currentHandPosition.getZ();});
    armTab.add("Gripper State", gripperControl);

    armTab.addNumber("Elbow Angle", () -> { return currentElbowAngle.getDegrees();});
    armTab.addNumber("Elbow Absolute Angle", () -> { return normalizeElbowAngle(elbowAngle.getAbsolutePosition());});

    armTab.addNumber("Shoulder Angle", () -> {return currentShoulderAngle.getDegrees();});
    armTab.addNumber("Shoulder Absolute Angle", () -> {return normalizeShoulderAngle(shoulderAngle.getAbsolutePosition());});
    armTab.addNumber("Shoulder Target Speed", () -> {return targetShoulderSpeed.getDegrees();});
    armTab.addNumber("Elbow Target Speed", () -> {return targetElbowSpeed.getDegrees();});
    
  }

  public static Translation3d calculateKinematics(double shoulderThetaDegrees, double elbowThetaDegrees) {
    double sTR = shoulderThetaDegrees * Math.PI / 180.0;
    double eTR = elbowThetaDegrees * Math.PI / 180.0;
    double tTR = sTR + eTR;

    double x = Constants.APOLLO_LENGTH * Math.cos(tTR) + Constants.ARTEMIS_LENGTH * Math.cos(sTR);
    double z = Constants.HERCULES_LENGTH + Constants.APOLLO_LENGTH * Math.sin(tTR) + Constants.ARTEMIS_LENGTH * Math.sin(sTR);

    return new Translation3d(x, 0.0, z);
  }

  public void setEndEffectorSpeeds(double xSpeed, double zSpeed) {
    JointSpeeds joints = calculateJointSpeeds(xSpeed, zSpeed);

    targetElbowSpeed = joints.getElbowSpeed();
    targetShoulderSpeed = joints.getShoulderSpeed();
  }

  public void setJointSpeeds(double shoulderSpeedRadians, double elbowSpeedRadians) {
    targetShoulderSpeed = Rotation2d.fromRadians(shoulderSpeedRadians);
    targetElbowSpeed = Rotation2d.fromRadians(elbowSpeedRadians);
  }

  public static List<Rotation2d> calculateInverseKinematics(double x, double z) {
    ArrayList<Rotation2d> returnList = new ArrayList<Rotation2d>(2);
    double zPrime = z - Constants.HERCULES_LENGTH;

    double elbow = Math.acos(elbowComponentCalc(x, zPrime));
    double shoulder = Math.atan2(zPrime, x) - Math.atan2(Constants.APOLLO_LENGTH * Math.sin(elbow), Constants.ARTEMIS_LENGTH + Constants.APOLLO_LENGTH * Math.cos(elbow));

    returnList.add(new Rotation2d(shoulder));
    returnList.add(new Rotation2d(elbow));

    elbow = -1 * Math.acos(elbowComponentCalc(x, zPrime));
    shoulder = Math.atan2(zPrime, x) - Math.atan2(Constants.APOLLO_LENGTH * Math.sin(elbow), Constants.ARTEMIS_LENGTH + Constants.APOLLO_LENGTH * Math.cos(elbow));

    returnList.add(new Rotation2d(shoulder));
    returnList.add(new Rotation2d(elbow));

    return returnList;
  }

  private static double elbowComponentCalc(double x, double zPrime) {
    return (Math.pow(x, 2.0) + Math.pow(zPrime, 2.0) - (Math.pow(Constants.ARTEMIS_LENGTH, 2.0) + Math.pow(Constants.APOLLO_LENGTH, 2.0))) / (2.0 * Constants.APOLLO_LENGTH * Constants.ARTEMIS_LENGTH);
  }

  private JointSpeeds calculateJointSpeeds(double xSpeed, double ySpeed) {
    double elbowXPartial = 0.0;
    double elbowYPartial = 0.0;
    double shoulderXPartial = 0.0;
    double shoulderYPartial = 0.0;
    double zPrime = currentHandPosition.getZ() - Constants.HERCULES_LENGTH;

    if (currentElbowAngle.getRadians() >= 0.0) {
      elbowXPartial = (-1.0 / Math.sqrt(1 - Math.pow(elbowComponentCalc(currentHandPosition.getX(), zPrime), 2.0))) * (currentHandPosition.getX() / (Constants.ARTEMIS_LENGTH * Constants.APOLLO_LENGTH));
      elbowYPartial = (-1.0 / Math.sqrt(1 - Math.pow(elbowComponentCalc(currentHandPosition.getX(), zPrime), 2.0))) * (zPrime / (Constants.ARTEMIS_LENGTH * Constants.APOLLO_LENGTH));
    } else {
      elbowXPartial = (1.0 / Math.sqrt(1 - Math.pow(elbowComponentCalc(currentHandPosition.getX(), zPrime), 2.0))) * (currentHandPosition.getX() / (Constants.ARTEMIS_LENGTH * Constants.APOLLO_LENGTH));
      elbowYPartial = (1.0 / Math.sqrt(1 - Math.pow(elbowComponentCalc(currentHandPosition.getX(), zPrime), 2.0))) * (currentHandPosition.getZ() / (Constants.ARTEMIS_LENGTH * Constants.APOLLO_LENGTH));
    }

    shoulderXPartial = ((-zPrime / Math.pow(currentHandPosition.getX(), 2.0)) / (1 + Math.pow((zPrime / currentHandPosition.getX()), 2.0))) * (((Constants.ARTEMIS_LENGTH + Constants.APOLLO_LENGTH * currentElbowAngle.getCos()) * (Constants.APOLLO_LENGTH * currentElbowAngle.getCos() * elbowXPartial)) - (Constants.APOLLO_LENGTH * currentElbowAngle.getSin()) * (-Constants.APOLLO_LENGTH * currentElbowAngle.getSin() * elbowXPartial)) / Math.pow(Constants.ARTEMIS_LENGTH + Constants.APOLLO_LENGTH * currentElbowAngle.getCos(), 2.0);
    shoulderYPartial = ((1.0 / (1.0 + Math.pow(zPrime / currentHandPosition.getX(), 2.0))) * (1.0 / currentHandPosition.getX())) - ((1.0 / (1.0 + Math.pow((Constants.APOLLO_LENGTH * currentElbowAngle.getSin()) / (Constants.ARTEMIS_LENGTH + Constants.APOLLO_LENGTH * currentElbowAngle.getCos()), 2.0))) * (((Constants.ARTEMIS_LENGTH + Constants.APOLLO_LENGTH * currentElbowAngle.getCos())*(Constants.APOLLO_LENGTH*currentElbowAngle.getCos()*elbowYPartial)-(Constants.APOLLO_LENGTH*currentElbowAngle.getSin())*(-Constants.APOLLO_LENGTH*currentElbowAngle.getSin()*elbowYPartial)) / Math.pow((Constants.ARTEMIS_LENGTH + Constants.APOLLO_LENGTH * currentElbowAngle.getCos()), 2.0)));

    double elbowSpeed = xSpeed * elbowXPartial + ySpeed * elbowYPartial;
    double shoulderSpeed = xSpeed * shoulderXPartial + ySpeed * shoulderYPartial;

    return new JointSpeeds(elbowSpeed, shoulderSpeed);
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
  targetShoulderSpeed = Constants.TESTING_SPEED;
//  shoulderControl.set(ControlMode.PercentOutput, targetShoulderSpeed.getDegrees());
}
public void moveshoulderdown (){
 targetShoulderSpeed = Constants.TESTING_SPEED.unaryMinus();
// shoulderControl.set(ControlMode.PercentOutput, targetShoulderSpeed.getDegrees());

}
public void stopshoulder (){
  shoulderControl.stopMotor();
  targetShoulderSpeed = Rotation2d.fromDegrees(0.0);
}



public void moveelbowup (){
  targetElbowSpeed = Constants.TESTING_SPEED;
// elbowControl.set(targetElbowSpeed.getDegrees());

}
public void elbowdown (){
  targetElbowSpeed = Constants.TESTING_SPEED.unaryMinus();
//  elbowControl.set(targetElbowSpeed.getDegrees());


}
public void stopelbow (){
  elbowControl.stopMotor();
  targetElbowSpeed = Rotation2d.fromDegrees(0.0);

}
public void stopArm () {
  stopelbow();
  stopshoulder();
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateState();
    double shoulderMotor = shoulderSpeedController.calculate(shoulderAngle.getVelocity(), targetShoulderSpeed.getDegrees());
    double elbowMotor = elbowSpeedController.calculate(elbowAngle.getVelocity(), targetElbowSpeed.getDegrees());

    double desiredShoulderSpeed =  targetShoulderSpeed.getRadians()/Constants.MAX_SHOULDER_SPEED.getRadians() + shoulderMotor;
    double desiredElbowSpeed = targetElbowSpeed.getRadians()/Constants.MAX_ELBOW_SPEED.getRadians() + elbowMotor;

    if ((currentShoulderAngle.getDegrees() >= 255.0)) {
      if (desiredShoulderSpeed > 0.0) {
        desiredShoulderSpeed = 0.0;
      }
    } else if (currentShoulderAngle.getDegrees() <= 10) {
      if (desiredShoulderSpeed < 0.0) {
        desiredShoulderSpeed = 0.0;
      }
    }

    if (currentElbowAngle.getDegrees() < -165.0) {
      if (desiredElbowSpeed < 0.0) {
        desiredElbowSpeed = 0.0;
      } 
    } else if (currentElbowAngle.getDegrees() > 165.0) {
      if (desiredElbowSpeed > 0.0) {
        desiredElbowSpeed = 0.0;
      } 
    
    

    }

     shoulderControl.set(ControlMode.PercentOutput, desiredShoulderSpeed);
     elbowControl.set(desiredElbowSpeed);
    SmartDashboard.putNumber("shoulderDemand", desiredShoulderSpeed);
    SmartDashboard.putNumber("ElbowDemand", desiredElbowSpeed);



   // shoulderControl.set(ControlMode.PercentOutput, shoulderMotor);
   // elbowControl.set(elbowMotor);

  }

  private void updateState() {
    currentShoulderAngle = Rotation2d.fromDegrees(normalizeShoulderAngle(shoulderAngle.getPosition()));
    currentElbowAngle = Rotation2d.fromDegrees(normalizeElbowAngle(elbowAngle.getPosition()));
    currentHandPosition = calculateKinematics(currentShoulderAngle.getDegrees(), currentElbowAngle.getDegrees());
  }

  private double normalizeElbowAngle(double angleInDegrees) {
    if (angleInDegrees > 180.0) {
      return angleInDegrees - 360.0;
    }

    if (angleInDegrees < -180.0) {
      return angleInDegrees + 360.0;
    }

    return angleInDegrees;
  }
  private double normalizeShoulderAngle(double angleInDegrees) {
    if (angleInDegrees > 360.0) {
      return angleInDegrees - 360.0;
    }

    if (angleInDegrees < 0) {
      return angleInDegrees + 360.0;
    }

    return angleInDegrees;
  }

  public Rotation2d getShoulderPosition() {
    return currentShoulderAngle;
  }
  public Rotation2d getElbowPosition() {
    return currentElbowAngle;
  }
}
