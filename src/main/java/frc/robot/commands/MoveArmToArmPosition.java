// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.LinkedList;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmPositions;

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
    armPathway = RobotContainer.m_arm.getPath(targetArmPosition);
    currentTarget = armPathway.peekFirst();
    goToNextPoint();
    if (currentTarget == null || RobotContainer.m_arm.getArmPosition() == ArmPositions.UNKNOWN) {
      initializationFailed = true;
    } else {
      initializationFailed = false;
    }
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
