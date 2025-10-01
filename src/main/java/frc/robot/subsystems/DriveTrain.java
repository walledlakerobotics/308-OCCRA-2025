// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.pathplanner.lib.auto.AutoBuilder;
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
    private final TalonSRX m_frontLeftMotor = new TalonSRX(DriveConstants.kFrontLeftMotorId);
    private final TalonSRX m_backLeftMotor = new TalonSRX(DriveConstants.kBackLeftMotorId);
    private final TalonSRX m_frontRightMotor = new TalonSRX(DriveConstants.kFrontRightMotorId);
    private final TalonSRX m_backRightMotor = new TalonSRX(DriveConstants.kBackRightMotorId);

    private final TalonSRX m_frontLeftMotorFollower = new TalonSRX(DriveConstants.kFrontLeftMotorFollowerId);
    private final TalonSRX m_backLeftMotorFollower = new TalonSRX(DriveConstants.kBackLeftMotorFollowerId);
    private final TalonSRX m_frontRightMotorFollower = new TalonSRX(DriveConstants.kFrontRightMotorFollowerId);
    private final TalonSRX m_backRightMotorFollower = new TalonSRX(DriveConstants.kBackRightMotorFollowerId);

    // calculates the wheel speeds based on the inputs
    private final MecanumDrive m_drive;

    // gyro sensor
    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

    // calculates odometry
    private final MecanumDriveOdometry m_odometry;

    // displays robot position on field
    private final Field2d m_field = new Field2d();

    private ShuffleboardTab m_driveTab = Shuffleboard.getTab(getName());

    /**
     * Constructs a {@link DriveTrain}.
     */
    public DriveTrain() {
        TalonSRXConfiguration config = new TalonSRXConfiguration();

        config.slot0.kP = DriveConstants.kVelocityP;
        config.slot0.kI = DriveConstants.kVelocityI;
        config.slot0.kD = DriveConstants.kVelocityD;

        config.continuousCurrentLimit = DriveConstants.kContinuousCurrentLimit;

        // left side motord
        m_frontLeftMotor.configAllSettings(config);
        m_backLeftMotor.configAllSettings(config);

        m_frontLeftMotor.setInverted(DriveConstants.kLeftMotorsInverted);
        m_backLeftMotor.setInverted(DriveConstants.kLeftMotorsInverted);

        m_frontLeftMotorFollower.setInverted(DriveConstants.kLeftMotorsInverted);
        m_backLeftMotorFollower.setInverted(DriveConstants.kLeftMotorsInverted);

        // right side motors=
        m_frontRightMotor.configAllSettings(config);
        m_backRightMotor.configAllSettings(config);

        m_frontRightMotor.setInverted(DriveConstants.kRightMotorsInverted);
        m_backRightMotor.setInverted(DriveConstants.kRightMotorsInverted);

        m_frontRightMotorFollower.setInverted(DriveConstants.kRightMotorsInverted);
        m_backRightMotorFollower.setInverted(DriveConstants.kRightMotorsInverted);

        m_frontLeftMotor.setNeutralMode(DriveConstants.kMotorIdleMode);
        m_backLeftMotor.setNeutralMode(DriveConstants.kMotorIdleMode);
        m_frontRightMotor.setNeutralMode(DriveConstants.kMotorIdleMode);
        m_backRightMotor.setNeutralMode(DriveConstants.kMotorIdleMode);

        m_frontLeftMotorFollower.setNeutralMode(DriveConstants.kMotorIdleMode);
        m_backLeftMotorFollower.setNeutralMode(DriveConstants.kMotorIdleMode);
        m_frontRightMotorFollower.setNeutralMode(DriveConstants.kMotorIdleMode);
        m_backRightMotorFollower.setNeutralMode(DriveConstants.kMotorIdleMode);

        m_frontLeftMotorFollower.follow(m_frontLeftMotor);
        m_backLeftMotorFollower.follow(m_backLeftMotor);
        m_frontRightMotorFollower.follow(m_frontRightMotor);
        m_backRightMotorFollower.follow(m_backRightMotor);

        m_drive = new MecanumDrive(
                speed -> m_frontLeftMotor.set(ControlMode.Velocity,
                        speed / DriveConstants.kRawPer100msToMetersPerSecond),
                speed -> m_backLeftMotor.set(ControlMode.Velocity,
                        speed / DriveConstants.kRawPer100msToMetersPerSecond),
                speed -> m_frontRightMotor.set(ControlMode.Velocity,
                        speed / DriveConstants.kRawPer100msToMetersPerSecond),
                speed -> m_backRightMotor.set(ControlMode.Velocity,
                        speed / DriveConstants.kRawPer100msToMetersPerSecond));

        m_odometry = new MecanumDriveOdometry(DriveConstants.kDriveKinematics,
                m_gyro.getRotation2d(),
                getWheelPositions());

        // we handle the deadband ourselves
        m_drive.setDeadband(0);

        // we want drive to pass speed as meters per second
        m_drive.setMaxOutput(DriveConstants.kMaxSpeedMetersPerSecond);

        m_driveTab.addNumber("Robot X (m)", () -> m_odometry.getPoseMeters().getX());
        m_driveTab.addNumber("Robot Y (m)", () -> m_odometry.getPoseMeters().getY());
        m_driveTab.addNumber("Robot Heading (deg)", () -> m_odometry.getPoseMeters().getRotation().getDegrees());

        m_driveTab.add("Field", m_field);

        AutoBuilder.configure(m_odometry::getPoseMeters, this::resetOdometry,
                this::getChassisSpeeds,
                this::driveRobotRelative, AutoConstants.kAutoController,
                AutoConstants.kRobotConfig, () -> false, this);
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

        m_frontLeftMotor.set(ControlMode.Velocity, wheelSpeeds.frontLeftMetersPerSecond
                / DriveConstants.kRawPer100msToMetersPerSecond);
        m_backLeftMotor.set(ControlMode.Velocity, wheelSpeeds.rearLeftMetersPerSecond
                / DriveConstants.kRawPer100msToMetersPerSecond);
        m_frontRightMotor.set(ControlMode.Velocity, wheelSpeeds.frontRightMetersPerSecond
                / DriveConstants.kRawPer100msToMetersPerSecond);
        m_backRightMotor.set(ControlMode.Velocity, wheelSpeeds.rearRightMetersPerSecond
                / DriveConstants.kRawPer100msToMetersPerSecond);
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
                m_frontLeftMotor.getSelectedSensorVelocity() * DriveConstants.kRawPer100msToMetersPerSecond,
                m_frontRightMotor.getSelectedSensorVelocity() * DriveConstants.kRawPer100msToMetersPerSecond,
                m_backLeftMotor.getSelectedSensorVelocity() * DriveConstants.kRawPer100msToMetersPerSecond,
                m_backRightMotor.getSelectedSensorVelocity() * DriveConstants.kRawPer100msToMetersPerSecond);

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
                m_frontLeftMotor.getSelectedSensorPosition() * DriveConstants.kRawToMeters,
                m_frontRightMotor.getSelectedSensorPosition() * DriveConstants.kRawToMeters,
                m_backLeftMotor.getSelectedSensorPosition() * DriveConstants.kRawToMeters,
                m_backRightMotor.getSelectedSensorPosition() * DriveConstants.kRawToMeters);
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
     * Sets the {@link NeutralMode} for all drivetrain motors. This will not persist
     * through power cycles.
     * 
     * @param mode The mode to set.
     */
    public void setIdleMode(NeutralMode mode) {
        for (TalonSRX motor : new TalonSRX[] { m_frontLeftMotor, m_backLeftMotor, m_frontRightMotor,
                m_backLeftMotor }) {
            motor.setNeutralMode(mode);
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
