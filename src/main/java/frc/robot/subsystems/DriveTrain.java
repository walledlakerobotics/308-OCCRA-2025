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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // gyro sensor
    private final AHRS m_gyro = new AHRS(NavXComType.kUSB1);

    // calculates odometry
    private final MecanumDriveOdometry m_odometry;

    // displays robot position on field
    private final Field2d m_field = new Field2d();

    private ShuffleboardTab m_driveTab = Shuffleboard.getTab(getName());

    private Rotation2d m_fieldRelativeOffset = Rotation2d.kZero;

    private Rotation2d m_rotationSetpoint = m_gyro.getRotation2d();
    private PIDController m_rotationController = new PIDController(DriveConstants.kRotationP, DriveConstants.kRotationI,
            DriveConstants.kRotationD);
    private boolean m_wasZRotationZero = false;

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

        // right side motors
        m_frontRightMotor.configAllSettings(config);
        m_backRightMotor.configAllSettings(config);

        m_frontRightMotor.setInverted(DriveConstants.kRightMotorsInverted);
        m_backRightMotor.setInverted(DriveConstants.kRightMotorsInverted);

        m_frontRightMotorFollower.setInverted(DriveConstants.kRightMotorsInverted);
        m_backRightMotorFollower.setInverted(DriveConstants.kRightMotorsInverted);

        m_frontLeftMotorFollower.configAllSettings(config);
        m_frontRightMotorFollower.configAllSettings(config);
        m_backLeftMotorFollower.configAllSettings(config);
        m_backRightMotorFollower.configAllSettings(config);

        m_frontLeftMotor.setNeutralMode(DriveConstants.kMotorNeutralMode);
        m_backLeftMotor.setNeutralMode(DriveConstants.kMotorNeutralMode);
        m_frontRightMotor.setNeutralMode(DriveConstants.kMotorNeutralMode);
        m_backRightMotor.setNeutralMode(DriveConstants.kMotorNeutralMode);

        m_frontLeftMotorFollower.setNeutralMode(DriveConstants.kMotorNeutralMode);
        m_backLeftMotorFollower.setNeutralMode(DriveConstants.kMotorNeutralMode);
        m_frontRightMotorFollower.setNeutralMode(DriveConstants.kMotorNeutralMode);
        m_backRightMotorFollower.setNeutralMode(DriveConstants.kMotorNeutralMode);

        m_frontLeftMotorFollower.follow(m_frontLeftMotor);
        m_backLeftMotorFollower.follow(m_backLeftMotor);
        m_frontRightMotorFollower.follow(m_frontRightMotor);
        m_backRightMotorFollower.follow(m_backRightMotor);

        m_odometry = new MecanumDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d(),
                getWheelPositions());

        m_driveTab.addNumber("Robot X (m)", () -> m_odometry.getPoseMeters().getX());
        m_driveTab.addNumber("Robot Y (m)", () -> m_odometry.getPoseMeters().getY());
        m_driveTab.addNumber("Robot Heading (deg)",
                () -> m_odometry.getPoseMeters().getRotation().getDegrees());

        m_driveTab.add("Field", m_field);

        m_rotationController.enableContinuousInput(0, 2 * Math.PI);

        AutoBuilder.configure(m_odometry::getPoseMeters, this::resetOdometry,
                this::getChassisSpeeds,
                this::drive, AutoConstants.kAutoController,
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
        xSpeed *= DriveConstants.kMaxForwardSpeedMetersPerSecond;
        ySpeed *= DriveConstants.kMaxStrafeSpeedMetersPerSecond;
        zRotation *= DriveConstants.kMaxRotationSpeedRadiansPerSecond;

        if (Math.abs(zRotation) < 0.05) {
            if (!m_wasZRotationZero) {
                m_rotationSetpoint = m_gyro.getRotation2d();
                m_wasZRotationZero = true;
            }

            zRotation = m_rotationController.calculate(m_gyro.getRotation2d().getRadians(),
                    m_rotationSetpoint.getRadians());
        } else {
            m_wasZRotationZero = false;
        }

        SmartDashboard.putNumber("Field Relative",
                m_gyro.getRotation2d().minus(m_fieldRelativeOffset).getDegrees());

        drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zRotation,
                m_gyro.getRotation2d().minus(m_fieldRelativeOffset)));
    }

    /**
     * Drives the robot based on robot relative {@link ChassisSpeeds}.
     * 
     * @param speeds
     */
    public void drive(ChassisSpeeds speeds) {
        SmartDashboard.putNumber("X Speed", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Y Speed", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("R Speed", speeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("R Setpoint", m_rotationSetpoint.getRadians());

        MecanumDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(speeds);

        SmartDashboard.putNumber("Front Left", wheelSpeeds.frontLeftMetersPerSecond);
        SmartDashboard.putNumber("Front Left Pos", getWheelPositions().frontLeftMeters);
        SmartDashboard.putNumber("Conversion", DriveConstants.kRawPer100msToMetersPerSecond);

        SmartDashboard.putNumber("Front Left Volt", m_frontLeftMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("Front Right Volt", m_frontRightMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("Rear Left Volt", m_backLeftMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("Rear Right Volt", m_backRightMotor.getMotorOutputVoltage());

        SmartDashboard.putNumber("Front Left Follower Volt", m_frontLeftMotorFollower.getMotorOutputVoltage());
        SmartDashboard.putNumber("Front Right Follower Volt", m_frontRightMotorFollower.getMotorOutputVoltage());
        SmartDashboard.putNumber("Rear Left Follower Volt", m_backLeftMotorFollower.getMotorOutputVoltage());
        SmartDashboard.putNumber("Rear Right Follower Volt", m_backRightMotorFollower.getMotorOutputVoltage());

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
    public Command drive(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
            DoubleSupplier zRotationSupplier) {
        return runOnce(this::resetRotationCorrection).andThen(run(() -> {
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
        }));
    }

    /**
     * Stops the robot.
     */
    public void stopDrive() {
        resetRotationCorrection();
        drive(0, 0, 0);
    }

    /**
     * Gets the current robot relative {@link ChassisSpeeds}.
     * 
     * @return The current robot relative chassis speeds.
     */
    public ChassisSpeeds getChassisSpeeds() {
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
                -m_frontLeftMotor.getSelectedSensorVelocity()
                        * DriveConstants.kRawPer100msToMetersPerSecond,
                -m_frontRightMotor.getSelectedSensorVelocity()
                        * DriveConstants.kRawPer100msToMetersPerSecond,
                -m_backLeftMotor.getSelectedSensorVelocity()
                        * DriveConstants.kRawPer100msToMetersPerSecond,
                -m_backRightMotor.getSelectedSensorVelocity()
                        * DriveConstants.kRawPer100msToMetersPerSecond);

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
                -m_frontLeftMotor.getSelectedSensorPosition() * DriveConstants.kRawToMeters,
                -m_frontRightMotor.getSelectedSensorPosition() * DriveConstants.kRawToMeters,
                -m_backLeftMotor.getSelectedSensorPosition() * DriveConstants.kRawToMeters,
                -m_backRightMotor.getSelectedSensorPosition() * DriveConstants.kRawToMeters);
    }

    /**
     * Resets the current tracked robot position to the specified pose.
     * 
     * @param pose The {@link Pose2d} to reset the position to.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(m_gyro.getRotation2d(), getWheelPositions(), pose);
    }

    private void resetRotationCorrection() {
        m_rotationSetpoint = m_gyro.getRotation2d();
        m_rotationController.reset();
        m_wasZRotationZero = false;
    }

    /**
     * Sets the {@link NeutralMode} for all drive train motors.
     * 
     * @param mode The neutral mode to set.
     */
    public void setNeutralMode(NeutralMode mode) {
        m_frontLeftMotor.setNeutralMode(DriveConstants.kMotorNeutralMode);
        m_backLeftMotor.setNeutralMode(DriveConstants.kMotorNeutralMode);
        m_frontRightMotor.setNeutralMode(DriveConstants.kMotorNeutralMode);
        m_backRightMotor.setNeutralMode(DriveConstants.kMotorNeutralMode);

        m_frontLeftMotorFollower.setNeutralMode(DriveConstants.kMotorNeutralMode);
        m_backLeftMotorFollower.setNeutralMode(DriveConstants.kMotorNeutralMode);
        m_frontRightMotorFollower.setNeutralMode(DriveConstants.kMotorNeutralMode);
        m_backRightMotorFollower.setNeutralMode(DriveConstants.kMotorNeutralMode);
    }

    public Command resetFieldRelative() {
        return runOnce(() -> {
            if (DriverStation.isFMSAttached())
                return;

            m_fieldRelativeOffset = m_gyro.getRotation2d();
        }).ignoringDisable(true);
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
