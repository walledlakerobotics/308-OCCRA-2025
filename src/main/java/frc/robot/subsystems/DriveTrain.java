// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.ControllerUtils;

/**
 * A subsystem that controls the robot's drivetrain.
 */
public class DriveTrain extends SubsystemBase {
    // left motors
    private final SparkMax m_leftLeader = new SparkMax(DriveConstants.kFrontLeftMotorId, MotorType.kBrushless);
    private final SparkMax m_leftFollower = new SparkMax(DriveConstants.kBackLeftMotorId, MotorType.kBrushless);

    // right motors
    private final SparkMax m_rightLeader = new SparkMax(DriveConstants.kFrontRightMotorId, MotorType.kBrushless);
    private final SparkMax m_rightFollower = new SparkMax(DriveConstants.kBackRightMotorId, MotorType.kBrushless);

    // encoders
    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;

    // closed loop (pid) controllers
    private final SparkClosedLoopController m_leftClosedLoop;
    private final SparkClosedLoopController m_rightClosedLoop;

    // gyro
    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

    // calculates the wheel speeds based on the inputs
    private final DifferentialDrive m_drive = new DifferentialDrive(this::setLeftVelocity, this::setRightVelocity);

    // calculates odometry
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), getLeftPosition(), getRightPosition());

    private ShuffleboardTab m_driveTab = Shuffleboard.getTab(getName());

    /**
     * Constructs a {@link DriveTrain}.
     */
    public DriveTrain() {
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

        // we want the speeds passed into the set speed functions to be in meters per
        // second
        m_drive.setMaxOutput(DriveConstants.kMaxSpeedMetersPerSecond);

        AutoBuilder.configure(m_odometry::getPoseMeters, this::resetOdometry, this::getChassisSpeeds,
                this::driveRobotRelative,
                AutoConstants.kAutoController, AutoConstants.kRobotConfig,
                () -> false, this);
    }

    /**
     * Drives the robot using curvature drive.
     * 
     * @param xSpeed           The robot's speed along the X axis [-1.0..1.0].
     *                         Forward is positive.
     * @param zRotation        The normalized curvature [-1.0..1.0].
     *                         Counterclockwise is positive.
     * @param allowTurnInPlace If set, overrides constant-curvature turning for
     *                         turn-in-place maneuvers. zRotation will control
     *                         turning rate instead of curvature.
     */
    public void drive(double forward, double curvature, boolean allowTurnInPlace) {
        m_drive.curvatureDrive(forward, curvature, allowTurnInPlace);
    }

    /**
     * Drives the robot based on raw robot relative {@link ChassisSpeeds}.
     * 
     * @param speeds
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(speeds);

        setLeftVelocity(wheelSpeeds.leftMetersPerSecond);
        setRightVelocity(wheelSpeeds.rightMetersPerSecond);
    }

    /**
     * Creates a {@link Command} that drives the robot based on joystick or axis
     * inputs.
     * 
     * @param xSpeedSupplier      Supplies the axis value for forward/backward
     *                            movement in the range [-1, 1]. Forward is
     *                            positive.
     *                            It will be transformed based on the sensitivity,
     *                            deadband, and multiplier values.
     * @param zRotationSupplier   Supplies the axis value for rotation in the range
     *                            [-1, 1]. Counterclockwise is positive. It will be
     *                            transformed based on the sensitivity,
     *                            deadband, and multiplier values.
     * @param turnInPlaceSupplier Supplies whether to override curvature drive to
     *                            all for turn in place maneuvers.
     * 
     * @return A Command that drives the robot based on joystick inputs.
     */
    public Command driveJoysticks(DoubleSupplier xSpeedSupplier, DoubleSupplier zRotationSupplier,
            BooleanSupplier turnInPlaceSupplier) {
        return run(() -> {
            double forward = xSpeedSupplier.getAsDouble();
            double turning = zRotationSupplier.getAsDouble();

            forward = ControllerUtils.joystickTransform(forward, DriveConstants.kForwardAxisSensitvity,
                    DriveConstants.kDeadBand, DriveConstants.kForwardAxisMultiplier);
            turning = ControllerUtils.joystickTransform(turning, DriveConstants.kRotatonAxisSenitvity,
                    DriveConstants.kDeadBand, DriveConstants.kTurningAxisMultiplier);

            drive(forward, turning, turnInPlaceSupplier.getAsBoolean());
        });
    }

    /**
     * Stops the robot.
     */
    public void stopDrive() {
        m_drive.stopMotor();
    }

    /**
     * Gets the current robot relative {@link ChassisSpeeds}.
     * 
     * @return The current robot relative chassis speeds.
     */
    private ChassisSpeeds getChassisSpeeds() {
        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(getLeftVelocity(),
                getRightVelocity());
        ChassisSpeeds speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(wheelSpeeds);

        return speeds;
    }

    /**
     * Gets the distance traveled by the left side of the drivetrain in meters.
     * 
     * @return The left position in meters. Forward is positive.
     */
    public double getLeftPosition() {
        return m_leftEncoder.getPosition();
    }

    /**
     * Gets the current velocity of the left side of the drivetrain in meters per
     * second.
     * 
     * @return The left velocity in meters per second. Forward is positive.
     */
    public double getLeftVelocity() {
        return m_leftEncoder.getVelocity();
    }

    /**
     * Sets the velocity on the left side of the drivetrain in meters per second.
     * 
     * @speed The velocity to set in meters per second. Forward is positive.
     */
    public void setLeftVelocity(double speed) {
        m_leftClosedLoop.setReference(speed, ControlType.kVelocity);
    }

    /**
     * Gets the distance traveled by the right side of the drivetrain in meters.
     * 
     * @return The right position in meters. Forward is positive.
     */
    public double getRightPosition() {
        return m_rightEncoder.getPosition();
    }

    /**
     * Gets the current velocity of the right side of the drivetrain in meters per
     * second.
     * 
     * @return The right velocity in meters per second. Forward is positive.
     */
    public double getRightVelocity() {
        return m_rightEncoder.getVelocity();
    }

    /**
     * Sets the velocity on the right side of the drivetrain in meters per second.
     * 
     * @speed The velocity to set in meters per second. Forward is positive.
     */
    public void setRightVelocity(double speed) {
        m_rightClosedLoop.setReference(speed, ControlType.kVelocity);
    }

    /**
     * Resets the current tracked robot position to the specified pose.
     * 
     * @param pose The {@link Pose2d} to reset the position to.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(m_gyro.getRotation2d(), getLeftPosition(),
                getRightPosition(), pose);
    }

    /**
     * Sets the {@link IdleMode} for all drivetrain motors. This will not persist
     * through power cycles.
     * 
     * @param mode The mode to set.
     */
    public void setIdleMode(IdleMode mode) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(mode);

        for (SparkMax motor : new SparkMax[] { m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower }) {
            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    @Override
    public void periodic() {
        m_odometry.update(m_gyro.getRotation2d(), getLeftPosition(),
                getRightPosition());
    }
}
