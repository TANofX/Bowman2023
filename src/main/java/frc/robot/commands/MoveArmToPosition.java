// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.GreekArm;

public class MoveArmToPosition extends CommandBase {
  private double desiredElbowPosition;
  private double desiredShoulderPosition; 
  private static final double MAX_ANGULAR_VELOCITY = 25 * Math.PI/180.0;
  private static final double ANGULAR_ACCELERATION = 25 * Math.PI/180.0;
  private Timer shoulderTimer = new Timer();
  private Timer elbowTimer = new Timer();
  private double shoulderVelocity;
  private double elbowVelocity;
  private double shoulderAccelerationTime;
  private double elbowAccelerationTime;

  private double shoulderTime;
  private double elbowTime;

  private double shoulderDirection;
  private double elbowDirection;

  private boolean shoulderAccelerating;
  private boolean elbowAccelerating;


  /** Creates a new MoveArmToPosition. */
  public MoveArmToPosition(double shoulderAngleDegrees, double elbowAngleDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    desiredShoulderPosition = shoulderAngleDegrees;
      desiredElbowPosition = elbowAngleDegrees;
    

    addRequirements(RobotContainer.m_arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Rotation2d currentShoulderPosition = RobotContainer.m_arm.getShoulderPosition();
    Rotation2d currentElbowPosition = RobotContainer.m_arm.getElbowPosition();

    shoulderVelocity = 0.0;
    elbowVelocity = 0.0;

    shoulderTime = 3.0/2.0 * (desiredShoulderPosition - currentShoulderPosition.getDegrees()) * Math.PI/180.0 / MAX_ANGULAR_VELOCITY;
    elbowTime = 3.0/2.0 * (desiredElbowPosition - currentElbowPosition.getDegrees()) * Math.PI / 180 / MAX_ANGULAR_VELOCITY;

    shoulderTimer.reset();
    elbowTimer.reset();

    shoulderTimer.start();
    elbowTimer.start();

    shoulderAccelerationTime = shoulderTime / 2;
    elbowAccelerationTime = elbowTime / 2;

    shoulderDirection = Math.signum(shoulderTime);
    shoulderTime = Math.abs(shoulderTime);

    elbowDirection = Math.signum(elbowTime);
    elbowTime = Math.abs(elbowTime);

    shoulderAccelerating = true;
    elbowAccelerating = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoulderVelocity = Math.abs(shoulderVelocity);
    if (shoulderTimer.get() >= (shoulderTime - shoulderAccelerationTime)) {
      shoulderVelocity = MAX_ANGULAR_VELOCITY - ANGULAR_ACCELERATION * (shoulderTimer.get() - (shoulderTime - shoulderAccelerationTime));
    } else
      if (shoulderVelocity >= MAX_ANGULAR_VELOCITY) {
        shoulderVelocity = MAX_ANGULAR_VELOCITY;
    if (shoulderAccelerating == true) {
      shoulderAccelerationTime = shoulderTimer.get();
      shoulderAccelerating = false;
    }
  }
    else {
      shoulderVelocity = ANGULAR_ACCELERATION * shoulderTimer.get();
    }
    if (shoulderVelocity < 0.0) {
      shoulderVelocity = 0.0;
    }

    elbowVelocity = Math.abs(elbowVelocity);
    if (elbowTimer.get() >= (elbowTime - elbowAccelerationTime)) {
      elbowVelocity = MAX_ANGULAR_VELOCITY - ANGULAR_ACCELERATION * (elbowTimer.get() - (elbowTime - elbowAccelerationTime));
    } else
      if (elbowVelocity >= MAX_ANGULAR_VELOCITY) {
        elbowVelocity = MAX_ANGULAR_VELOCITY;
    if (elbowAccelerating == true) {
      elbowAccelerationTime = elbowTimer.get();
      elbowAccelerating = false;
    }
  }
    else {
      elbowVelocity = ANGULAR_ACCELERATION * elbowTimer.get();
    }
    if (elbowVelocity < 0.0) {
      elbowVelocity = 0.0;
    }

    RobotContainer.m_arm.setJointSpeeds(shoulderVelocity * shoulderDirection, elbowVelocity * elbowDirection);
  }
  
  

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_arm.setJointSpeeds(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shoulderTimer.hasElapsed(shoulderTime) && elbowTimer.hasElapsed(elbowTime);
    
}
}
