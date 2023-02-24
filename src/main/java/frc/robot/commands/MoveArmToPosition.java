// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.GreekArm;

public class MoveArmToPosition extends CommandBase {
private PIDController elbowController;
private PIDController shoulderController;
private Rotation2d targetShoulder;
private Rotation2d targetElbow;

  /** Creates a new MoveArmToPosition. */
  public MoveArmToPosition(double shoulderAngleDegrees, double elbowAngleDegrees) {
    targetElbow = Rotation2d.fromDegrees(elbowAngleDegrees);
    targetShoulder = Rotation2d.fromDegrees(shoulderAngleDegrees);

     // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_arm.setTargetAngles(targetShoulder.getDegrees(), targetElbow.getDegrees());
  }

      
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(RobotContainer.m_arm.getShoulderPosition().getDegrees() - targetShoulder.getDegrees()) < 5.0) && (Math.abs(RobotContainer.m_arm.getElbowPosition().getDegrees() - targetElbow.getDegrees()) < 5.0);
}
}
