package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;


import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private ChassisSpeeds chasSpeed = new ChassisSpeeds();
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

        ShuffleboardTab tab = Shuffleboard.getTab("Drive Control");
        tab.addNumber("X Input", m_translationXSupplier);
        tab.addNumber("Y Input", m_translationYSupplier);
        tab.addNumber("Omega Input", m_rotationSupplier);
        tab.addNumber("X Speed", () -> {return chasSpeed.vxMetersPerSecond;});
        tab.addNumber("Y Speed", () -> {return chasSpeed.vyMetersPerSecond;});
        tab.addNumber("Omega Speed", () -> {return chasSpeed.omegaRadiansPerSecond;});
    }

    @Override
    public void initialize() {
        maintainAngle.setSetpoint(m_drivetrainSubsystem.getGyroscopeRotation().getRadians());
    }

    @Override
    public void execute() {
        double xValue = m_translationXSupplier.getAsDouble()  * Constants.DRIVE_SPEED_PERCENTAGE * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        double yValue = m_translationYSupplier.getAsDouble()  * Constants.DRIVE_SPEED_PERCENTAGE * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        double omegaValue = m_rotationSupplier.getAsDouble()  *  Constants.DRIVE_SPEED_PERCENTAGE  * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        switch (DriverStation.getAlliance()) {
                    case Red:
                            xValue *= -1;
                            yValue *= -1;

                            break;
                    default:
                            break;
                 }
        // double driveFraction = m_drivetrainSubsystem.getdriveFraction();
        // double rotationFraction = m_drivetrainSubsystem.getrotationFraction();
        // chasSpeed = new ChassisSpeeds(driveFraction * m_translationXSupplier.getAsDouble() + (1 - driveFraction) * priorChasSpeed.vxMetersPerSecond
        //                                 ,driveFraction * m_translationYSupplier.getAsDouble() + (1 - driveFraction) * priorChasSpeed.vyMetersPerSecond
        //                                 ,rotationFraction * m_rotationSupplier.getAsDouble() + (1 - rotationFraction) * priorChasSpeed.omegaRadiansPerSecond);
        // // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        chasSpeed = 
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xValue,
                        yValue,
                        omegaValue,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                );
        if (Math.abs(m_rotationSupplier.getAsDouble()) > 0.015) {
            maintainAngle.setSetpoint(m_drivetrainSubsystem.getGyroscopeRotation().getRadians());
        }
        if ((Math.abs(m_translationXSupplier.getAsDouble() )> 0.05) || (Math.abs(m_translationYSupplier.getAsDouble()) > 0.05)) {
        chasSpeed.omegaRadiansPerSecond = chasSpeed.omegaRadiansPerSecond + maintainAngle.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getRadians());
        }
        m_drivetrainSubsystem.drive(chasSpeed);
        priorChasSpeed = chasSpeed;

    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
