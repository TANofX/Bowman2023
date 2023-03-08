// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ManualArm extends CommandBase {
  /** Creates a new ManualArm. */
  private DoubleSupplier shoulder;
  private DoubleSupplier elbow;
  public ManualArm(DoubleSupplier shoulderSupplier, DoubleSupplier elbowSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    shoulder = shoulderSupplier;
    elbow = elbowSupplier;

    addRequirements(RobotContainer.m_arm);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_arm.setJointSpeeds(shoulder.getAsDouble(), elbow.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_arm.setJointSpeeds(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
