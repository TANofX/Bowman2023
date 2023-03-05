// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autobalance extends PIDCommand {
  Timer finishTimer = new Timer();
  
  public enum BalancePoint {
     LEVEL(0.0), FORWARD(-11.0), BACKWARD(11.0);
   
  public final double setPoint;

  private BalancePoint(double val) {
     setPoint = val;
    }
  }

  /** Creates a new Autobalance. */
  public Autobalance(BalancePoint balance) {
    super(
        // The controller that the command will use
        new PIDController(0.025, 0, 0.00),
        // This should return the measurement
        () -> {return RobotContainer.m_drivetrainSubsystem.getPitch();},
        // This should return the setpoint (can also be a constant)
        () -> balance.setPoint,
        // This uses the output
        output -> {ChassisSpeeds pitchMovement =  new ChassisSpeeds(output, 0, 0);
                  RobotContainer.m_drivetrainSubsystem.drive(pitchMovement);
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(RobotContainer.m_drivetrainSubsystem);
    getController().setTolerance(1.75);

    finishTimer.start();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(getController().getPositionError()) > getController().getPositionTolerance()) {
      finishTimer.reset();
    }
    return finishTimer.get() > 1.0;

  }
}
