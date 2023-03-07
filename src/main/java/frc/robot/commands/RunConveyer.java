// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class RunConveyer extends CommandBase {
  private double speed;
  private DoubleSupplier speedSupplier = null;

  public RunConveyer() {
    addRequirements(RobotContainer.m_conveyer);
  }

  /** Creates a new RunConveyer. */
  public RunConveyer(double conveyerSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
   this();

    speed = conveyerSpeed;
  }
  public RunConveyer(DoubleSupplier ds) {
    this();
    speedSupplier = ds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (speedSupplier == null) {
      RobotContainer.m_conveyer.runConveyer(speed);
    }
    else {
      RobotContainer.m_conveyer.runConveyer(speedSupplier.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_conveyer.runConveyer(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
