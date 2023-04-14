// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.GreekArm;

public class MoveArmMeters extends CommandBase {
  private double xMetersToMove = 0.0;
  private double zMetersToMove = 0.0;

  private Rotation2d targetShoulder = new Rotation2d();
  private Rotation2d targetElbow = new Rotation2d();
  private double angleTolerance = 0.0;

  /** Creates a new MoveArmMeters. */
  public MoveArmMeters(double xMeters, double zMeters, double tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    xMetersToMove = xMeters;
    zMetersToMove = zMeters;
    angleTolerance = tolerance;
    addRequirements(RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Rotation2d elbowRotation = RobotContainer.m_arm.getElbowPosition();
    Rotation2d shoulderRotation = RobotContainer.m_arm.getShoulderPosition();

    Translation3d handPosition = GreekArm.calculateKinematics(shoulderRotation.getDegrees(), elbowRotation.getDegrees());

    Translation3d movement = new Translation3d(xMetersToMove, 0.0, zMetersToMove);

    Translation3d targetHandPosition = handPosition.plus(movement);

    List<Rotation2d> rotations = GreekArm.calculateInverseKinematics(targetHandPosition.getX(), targetHandPosition.getZ());
    targetShoulder = Rotation2d.fromDegrees(GreekArm.normalizeShoulderAngle(rotations.get(0).getDegrees()));
    targetElbow = Rotation2d.fromDegrees(GreekArm.normalizeElbowAngle(rotations.get(1).getDegrees()));

    SmartDashboard.putNumber("StartingShoulder", shoulderRotation.getDegrees());
    SmartDashboard.putNumber("Starting Elbow", elbowRotation.getDegrees());
    SmartDashboard.putNumber("Target Shoulder", targetShoulder.getDegrees());
    SmartDashboard.putNumber("Target Elbow", targetElbow.getDegrees());
    RobotContainer.m_arm.setTargetAngles(targetShoulder.getDegrees(), 
                                        targetElbow.getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((Math.abs(RobotContainer.m_arm.getShoulderPosition().getDegrees() - targetShoulder.getDegrees()) < angleTolerance) 
    && (Math.abs(RobotContainer.m_arm.getElbowPosition().getDegrees() - targetElbow.getDegrees()) < angleTolerance)) {
      return true;
    }

    return false;
  }
}
