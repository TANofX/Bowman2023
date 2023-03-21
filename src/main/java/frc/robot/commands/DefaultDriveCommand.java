package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private ChassisSpeeds chasSpeed;
    private ChassisSpeeds priorChasSpeed = new ChassisSpeeds();
    private PIDController maintainAngle;

    
    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
        maintainAngle = new PIDController(.75, 0, 0);
        maintainAngle.setSetpoint(0);
    }

    @Override
    public void execute() {
        
        // double driveFraction = m_drivetrainSubsystem.getdriveFraction();
        // double rotationFraction = m_drivetrainSubsystem.getrotationFraction();
        // chasSpeed = new ChassisSpeeds(driveFraction * m_translationXSupplier.getAsDouble() + (1 - driveFraction) * priorChasSpeed.vxMetersPerSecond
        //                                 ,driveFraction * m_translationYSupplier.getAsDouble() + (1 - driveFraction) * priorChasSpeed.vyMetersPerSecond
        //                                 ,rotationFraction * m_rotationSupplier.getAsDouble() + (1 - rotationFraction) * priorChasSpeed.omegaRadiansPerSecond);
        // // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        chasSpeed = 
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        m_drivetrainSubsystem.getGyroscopeRotation()
                );
        if (Math.abs(m_rotationSupplier.getAsDouble()) > 0.001) {
            maintainAngle.setSetpoint(m_drivetrainSubsystem.getGyroscopeRotation().getRadians());
        }
        chasSpeed.omegaRadiansPerSecond = chasSpeed.omegaRadiansPerSecond + maintainAngle.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getRadians());
        m_drivetrainSubsystem.drive(chasSpeed);
        priorChasSpeed = chasSpeed;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
