// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.ControllerUtils;

/*
 * This is still a WIP
 */

public class DriveSubsystem extends SubsystemBase {
    // this is where I make the lists of devices like motors and encoders, and
    // configs for the motors
    private final SparkMax m_leftLeader = new SparkMax(DriveConstants.kFrontLeftMotorId, MotorType.kBrushless);
    private final SparkMax m_leftFollower = new SparkMax(DriveConstants.kBackLeftMotorId, MotorType.kBrushless);

    private final SparkMax m_rightLeader = new SparkMax(DriveConstants.kFrontRightMotorId, MotorType.kBrushless);
    private final SparkMax m_rightFollower = new SparkMax(DriveConstants.kBackRightMotorId, MotorType.kBrushless);

    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;

    private final SparkClosedLoopController m_leftClosedLoop;
    private final SparkClosedLoopController m_rightClosedLoop;

    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

    private final DifferentialDrive m_drive = new DifferentialDrive(this::setLeftSpeed, this::setRightSpeed);

    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), getLeftPosition(), getRightPosition());

    private ShuffleboardTab m_driveTab = Shuffleboard.getTab(getName());

    public DriveSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();

        // sets the idle mode, the smart current limit, and the inversion
        config.smartCurrentLimit(DriveConstants.kSmartCurrentLimit);
        config.idleMode(DriveConstants.kMotorIdleMode);

        // sets the PID
        config.closedLoop.pid(DriveConstants.kVelocityP, DriveConstants.kVelocityI, DriveConstants.kVelocityD);
        config.closedLoop.velocityFF(DriveConstants.kVelocityFF);

        // sets encoder conversion factors
        config.encoder.positionConversionFactor(DriveConstants.kRotationsToMeters);
        config.encoder.velocityConversionFactor(DriveConstants.KRotationsPerMinuteToMetersPerSecond);

        // left side motors
        config.inverted(DriveConstants.kLeftMotorsInverted);

        m_leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(m_leftLeader.getDeviceId(), false);
        m_leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.disableFollowerMode();

        // right side motors
        config.inverted(DriveConstants.kRightMotorsInverted);

        m_rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(m_rightLeader.getDeviceId(), false);
        m_rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_leftEncoder = m_leftLeader.getEncoder();
        m_rightEncoder = m_rightLeader.getEncoder();

        m_leftClosedLoop = m_leftLeader.getClosedLoopController();
        m_rightClosedLoop = m_rightLeader.getClosedLoopController();

        // we handle the deadband ourselves
        m_drive.setDeadband(0);

        // we want the speeds passed into the set speed functions to be in meters per second
        m_drive.setMaxOutput(DriveConstants.kHardwareMaxSpeedMetersPerSecond);
    }

    /**
     * 
     * @param speeds
     */
    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(speeds);

        // write speeds to motors
        m_drive.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond, false);
    }

    /**
     * 
     * @param forward
     * @param turning
     */
    public void drive(double forward, double turning, boolean allowTurnInPlace) {
        m_drive.curvatureDrive(forward, turning, allowTurnInPlace);
    }

    /**
     * 
     * @param forwardSupplier
     * @param turningSupplier
     */
    public Command driveJoysticks(DoubleSupplier forwardSupplier, DoubleSupplier turningSupplier, BooleanSupplier turnInPlaceSupplier) {
        return run(() -> {
            double forward = forwardSupplier.getAsDouble();
            double turning = turningSupplier.getAsDouble();

            forward *= DriveConstants.kMaxForwardPercent;
            turning *= DriveConstants.kMaxTurningPercent;

            forward = ControllerUtils.sensitivity(forward, DriveConstants.kForwardSensitvity,
                    DriveConstants.kDeadBand);
            turning = ControllerUtils.sensitivity(turning, DriveConstants.kRotatonSenitvity,
                    DriveConstants.kDeadBand);

            drive(forward, turning, turnInPlaceSupplier.getAsBoolean());
        });
    }

    // stops all the motors
    public void stopDrive() {
        m_drive.stopMotor();
    }

    // sets all idle modes
    public void setAllIdleMode(IdleMode mode) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(mode);

        for (SparkMax motor : new SparkMax[] { m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower }) {
            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    private ChassisSpeeds getChassisSpeeds() {
        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
        ChassisSpeeds speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(wheelSpeeds);
        speeds.omegaRadiansPerSecond = Units.degreesToRadians(m_gyro.getRate());

        return speeds;
    }

    /**
     * 
     * @return
     */
    public double getLeftPosition() {
        return m_leftEncoder.getPosition();
    }

    /**
     * 
     * @return
     */
    public double getLeftSpeed() {
        return m_leftEncoder.getVelocity();
    }

    /**
     * 
     * @return
     */
    public void setLeftSpeed(double speed) {
        m_leftClosedLoop.setReference(speed, ControlType.kVelocity);
    }

    /**
     * 
     * @return
     */
    public double getRightPosition() {
        return m_rightEncoder.getPosition();
    }

    /**
     * 
     * @return
     */
    public double getRightSpeed() {
        return m_rightEncoder.getVelocity();
    }

    /**
     * 
     * @return
     */
    public void setRightSpeed(double speed) {
        m_rightClosedLoop.setReference(speed, ControlType.kVelocity);
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(m_gyro.getRotation2d(), getLeftPosition(),
                getRightPosition(), pose);
    }

    @Override
    public void periodic() {
        m_odometry.update(m_gyro.getRotation2d(), getLeftPosition(),
                getRightPosition());
    }
}
