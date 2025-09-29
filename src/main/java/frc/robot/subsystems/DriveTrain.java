// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.ControllerUtils;

/**
 * A subsystem that controls the robot's drive train.
 */
public class DriveTrain extends SubsystemBase {
    // drive train motors
    private final SparkMax m_frontLeftMotor = new SparkMax(DriveConstants.kFrontLeftMotorId, MotorType.kBrushless);
    private final SparkMax m_backLeftMotor = new SparkMax(DriveConstants.kBackLeftMotorId, MotorType.kBrushless);
    private final SparkMax m_frontRightMotor = new SparkMax(DriveConstants.kFrontRightMotorId, MotorType.kBrushless);
    private final SparkMax m_backRightMotor = new SparkMax(DriveConstants.kBackRightMotorId, MotorType.kBrushless);

    // encoders
    private final RelativeEncoder m_frontLeftEncoder;
    private final RelativeEncoder m_frontRightEncoder;
    private final RelativeEncoder m_backLeftEncoder;
    private final RelativeEncoder m_backRightEncoder;

    // closed loop (pid) controllers
    private final SparkClosedLoopController m_frontLeftClosedLoop;
    private final SparkClosedLoopController m_frontRightClosedLoop;
    private final SparkClosedLoopController m_backLeftClosedLoop;
    private final SparkClosedLoopController m_backRightClosedLoop;

    // gyro
    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

    // calculates the wheel speeds based on the inputs
    private final MecanumDrive m_drive;

    // calculates odometry
    private final MecanumDriveOdometry m_odometry;

    // displays robot position on field
    private final Field2d m_field = new Field2d();

    private ShuffleboardTab m_driveTab = Shuffleboard.getTab(getName());

    /**
     * Constructs a {@link DriveTrain}.
     */
    public DriveTrain() {
        SparkMaxConfig config = new SparkMaxConfig();

        // sets the idle mode, the smart current limit, and the inversion
        config
                .smartCurrentLimit(DriveConstants.kSmartCurrentLimitAmps)
                .idleMode(DriveConstants.kMotorIdleMode);

        // sets the PID
        config.closedLoop
                .pid(DriveConstants.kVelocityP, DriveConstants.kVelocityI, DriveConstants.kVelocityD)
                .velocityFF(DriveConstants.kVelocityFF);

        // sets encoder conversion factors
        config.encoder
                .positionConversionFactor(DriveConstants.kRotationsToMeters)
                .velocityConversionFactor(DriveConstants.KRotationsPerMinuteToMetersPerSecond);

        // left side motors
        config.inverted(DriveConstants.kLeftMotorsInverted);

        m_frontLeftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_backLeftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // right side motors
        config.inverted(DriveConstants.kRightMotorsInverted);

        m_frontRightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_backLeftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // get encoders
        m_frontLeftEncoder = m_frontLeftMotor.getEncoder();
        m_frontRightEncoder = m_frontRightMotor.getEncoder();
        m_backLeftEncoder = m_backLeftMotor.getEncoder();
        m_backRightEncoder = m_backRightMotor.getEncoder();

        // get closed loop controllers
        m_frontLeftClosedLoop = m_frontLeftMotor.getClosedLoopController();
        m_frontRightClosedLoop = m_frontRightMotor.getClosedLoopController();
        m_backLeftClosedLoop = m_backLeftMotor.getClosedLoopController();
        m_backRightClosedLoop = m_backRightMotor.getClosedLoopController();

        m_drive = new MecanumDrive(
                speed -> m_frontLeftClosedLoop.setReference(speed, ControlType.kVelocity),
                speed -> m_backLeftClosedLoop.setReference(speed, ControlType.kVelocity),
                speed -> m_frontRightClosedLoop.setReference(speed, ControlType.kVelocity),
                speed -> m_backRightClosedLoop.setReference(speed, ControlType.kVelocity));

        m_odometry = new MecanumDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d(),
                getWheelPositions());

        // we handle the deadband ourselves
        m_drive.setDeadband(0);

        // we want the speeds passed into the set speed functions to be in meters per
        // second
        m_drive.setMaxOutput(DriveConstants.kMaxSpeedMetersPerSecond);

        m_driveTab.addNumber("Robot X (m)", () -> m_odometry.getPoseMeters().getX());
        m_driveTab.addNumber("Robot Y (m)", () -> m_odometry.getPoseMeters().getY());
        m_driveTab.addNumber("Robot Heading (deg)", () -> m_odometry.getPoseMeters().getRotation().getDegrees());

        m_driveTab.add("Field", m_field);

        AutoBuilder.configure(m_odometry::getPoseMeters, this::resetOdometry, this::getChassisSpeeds,
                this::driveRobotRelative, AutoConstants.kAutoController, AutoConstants.kRobotConfig, () -> false, this);
    }

    /**
     * Drives the robot using curvature drive.
     * 
     * @param xSpeed    The robot's speed along the X axis [-1, 1].
     *                  Forward is positive.
     * @param ySpeed    The robot's speed along the Y axis [-1, 1].
     *                  Left is positive.
     * @param zRotation The normalized curvature [-1, 1].
     *                  Counterclockwise is positive.
     */
    public void drive(double xSpeed, double ySpeed, double zRotation) {
        m_drive.driveCartesian(xSpeed, ySpeed, zRotation, m_gyro.getRotation2d());
    }

    /**
     * Drives the robot based on raw robot relative {@link ChassisSpeeds}.
     * 
     * @param speeds
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(speeds);

        m_frontLeftClosedLoop.setReference(wheelSpeeds.frontLeftMetersPerSecond, ControlType.kVelocity);
        m_backLeftClosedLoop.setReference(wheelSpeeds.rearLeftMetersPerSecond, ControlType.kVelocity);
        m_frontRightClosedLoop.setReference(wheelSpeeds.frontRightMetersPerSecond, ControlType.kVelocity);
        m_backRightClosedLoop.setReference(wheelSpeeds.rearRightMetersPerSecond, ControlType.kVelocity);
    }

    /**
     * Creates a {@link Command} that drives the robot based on joystick or axis
     * inputs.
     * 
     * @param xSpeedSupplier    Supplies the axis value for forward/backward
     *                          movement in the range [-1, 1]. Back is
     *                          positive. It will be transformed based on the
     *                          sensitivity,
     *                          deadband, and multiplier values.
     * @param ySpeedSupplier    Supplies the axis value for left/right
     *                          movement in the range [-1, 1]. Right is positive.
     *                          It will be transformed based on the sensitivity,
     *                          deadband, and multiplier values.
     * @param zRotationSupplier Supplies the axis value for rotation in the range
     *                          [-1, 1]. Clockwise is positive. It will be
     *                          transformed based on the sensitivity,
     *                          deadband, and multiplier values.
     * 
     * @return A Command that drives the robot based on joystick inputs.
     */
    public Command driveJoysticks(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
            DoubleSupplier zRotationSupplier) {
        return run(() -> {
            double xSpeed = -xSpeedSupplier.getAsDouble();
            double ySpeed = -ySpeedSupplier.getAsDouble();
            double zRotation = -zRotationSupplier.getAsDouble();

            xSpeed = ControllerUtils.joystickTransform(xSpeed, DriveConstants.kXAxisSensitvity,
                    DriveConstants.kDeadBand, DriveConstants.kXAxisMultiplier);
            ySpeed = ControllerUtils.joystickTransform(ySpeed, DriveConstants.kYAxisSensitvity,
                    DriveConstants.kDeadBand, DriveConstants.kYAxisMultiplier);
            zRotation = ControllerUtils.joystickTransform(zRotation, DriveConstants.kRotationAxisSensitivity,
                    DriveConstants.kDeadBand, DriveConstants.kRotationAxisMultiplier);

            drive(xSpeed, ySpeed, zRotation);
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
    public ChassisSpeeds getChassisSpeeds() {
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
                m_frontLeftEncoder.getVelocity(),
                m_frontRightEncoder.getVelocity(),
                m_backLeftEncoder.getVelocity(),
                m_backRightEncoder.getVelocity());

        ChassisSpeeds speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(wheelSpeeds);

        return speeds;
    }

    /**
     * Gets the current wheel positions in meters.
     * 
     * @return The {@link MecanumDriveWheelPositions} representing the current wheel
     *         positions in meters.
     */
    public MecanumDriveWheelPositions getWheelPositions() {
        return new MecanumDriveWheelPositions(
                m_frontLeftEncoder.getPosition(),
                m_frontRightEncoder.getPosition(),
                m_backLeftEncoder.getPosition(),
                m_backRightEncoder.getPosition());
    }

    /**
     * Resets the current tracked robot position to the specified pose.
     * 
     * @param pose The {@link Pose2d} to reset the position to.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(m_gyro.getRotation2d(), getWheelPositions(), pose);
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

        for (SparkMax motor : new SparkMax[] { m_frontLeftMotor, m_backLeftMotor, m_frontRightMotor,
                m_backLeftMotor }) {
            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    @Override
    public void periodic() {
        m_odometry.update(m_gyro.getRotation2d(), getWheelPositions());

        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    @Override
    public String getName() {
        return "Drive Train";
    }
}
