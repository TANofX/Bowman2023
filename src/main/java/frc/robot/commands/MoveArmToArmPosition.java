// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmPositions;
import frc.robot.subsystems.GreekArm;

public class MoveArmToArmPosition extends CommandBase {
  private ArmPositions targetArmPosition = ArmPositions.UNKNOWN;
  private LinkedList<ArmPositions> armPathway = new LinkedList<ArmPositions>();
  private ArmPositions currentTarget = ArmPositions.UNKNOWN;
  private boolean lastPosition = false;
  private boolean initializationFailed = false;
  /** Creates a new MoveArmToArmPosition. */
  public MoveArmToArmPosition(ArmPositions targetArmPosition) {
    this.targetArmPosition = targetArmPosition;
    addRequirements(RobotContainer.m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if ((targetArmPosition == ArmPositions.SLAM_JAM) && (RobotContainer.m_arm.getArmPosition() == ArmPositions.MID_SCORE)) {
      targetArmPosition = ArmPositions.MID_SLAM_JAM;
    }
    armPathway = RobotContainer.m_arm.getPath(targetArmPosition);
    currentTarget = armPathway.peekFirst();
    goToNextPoint();
    if (currentTarget == null || RobotContainer.m_arm.getArmPosition() == ArmPositions.UNKNOWN) {
      initializationFailed = true;
    } else {
      initializationFailed = false;
    }

    Rotation2d elbowRotation = RobotContainer.m_arm.getElbowPosition();
    Rotation2d shoulderRotation = RobotContainer.m_arm.getShoulderPosition();

    Translation3d handPosition = GreekArm.calculateKinematics(shoulderRotation.getDegrees(), elbowRotation.getDegrees());

    Translation3d movement = new Translation3d(0.0, 0.0, -0.2);

    Translation3d targetHandPosition = handPosition.plus(movement);

    List<Rotation2d> rotations = GreekArm.calculateInverseKinematics(targetHandPosition.getX(), targetHandPosition.getZ());
    Rotation2d targetShoulder = Rotation2d.fromDegrees(GreekArm.normalizeShoulderAngle(rotations.get(0).getDegrees()));
    Rotation2d targetElbow = Rotation2d.fromDegrees(GreekArm.normalizeElbowAngle(rotations.get(1).getDegrees()));

    SmartDashboard.putNumber("StartingShoulder", shoulderRotation.getDegrees());
    SmartDashboard.putNumber("Starting Elbow", elbowRotation.getDegrees());
    SmartDashboard.putNumber("Target Shoulder", targetShoulder.getDegrees());
    SmartDashboard.putNumber("Target Elbow", targetElbow.getDegrees());
  }

  private void goToNextPoint() {
    ArmPositions newTarget = armPathway.poll();
    if (newTarget != null) {
      currentTarget = newTarget;
      lastPosition = (armPathway.size() == 0);
      RobotContainer.m_arm.setTargetAngles(currentTarget.shoulderRotation.getDegrees(), currentTarget.elbowRotation.getDegrees());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (initializationFailed) this.initialize();

    if ((RobotContainer.m_arm.getArmPosition() == currentTarget) && !lastPosition) {
      goToNextPoint();
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean AreWeFinished = false;
    try {
    if (lastPosition && currentTarget != null) {
      if((Math.abs(RobotContainer.m_arm.getShoulderPosition().getDegrees() - currentTarget.shoulderRotation.getDegrees()) < currentTarget.rotationTolerance.getDegrees()) 
      && (Math.abs(RobotContainer.m_arm.getElbowPosition().getDegrees() - currentTarget.elbowRotation.getDegrees()) < currentTarget.rotationTolerance.getDegrees())) {
        AreWeFinished = true;
      }
      
    }
  }
  catch (Exception e) {}
    return AreWeFinished;
  }
}
