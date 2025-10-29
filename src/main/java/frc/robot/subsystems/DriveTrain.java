// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
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
    private final SparkMax m_rearLeftMotor = new SparkMax(DriveConstants.kRearLeftMotorId, MotorType.kBrushless);
    private final SparkMax m_frontRightMotor = new SparkMax(DriveConstants.kFrontRightMotorId, MotorType.kBrushless);
    private final SparkMax m_rearRightMotor = new SparkMax(DriveConstants.kRearRightMotorId, MotorType.kBrushless);

    private final SparkMax[] m_allMotors = { m_frontLeftMotor, m_rearLeftMotor, m_frontRightMotor, m_rearRightMotor };

    // encoders
    private final RelativeEncoder m_frontLeftEncoder;
    private final RelativeEncoder m_rearLeftEncoder;
    private final RelativeEncoder m_frontRightEncoder;
    private final RelativeEncoder m_rearRightEncoder;

    // closed loop (pid) controllers
    private final SparkClosedLoopController m_frontLeftClosedLoop;
    private final SparkClosedLoopController m_rearLeftClosedLoop;
    private final SparkClosedLoopController m_frontRightClosedLoop;
    private final SparkClosedLoopController m_rearRightClosedLoop;

    // feedforward for drive
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
            DriveConstants.kVelocityS,
            DriveConstants.kVelocityV,
            DriveConstants.kVelocityA);

    // gyro
    private final AHRS m_gyro = new AHRS(NavXComType.kUSB1);

    // calculates odometry
    private final MecanumDriveOdometry m_odometry;

    // displays robot position on field
    private final Field2d m_field = new Field2d();

    private Rotation2d m_fieldRelativeOffset = Rotation2d.kZero;

    private Rotation2d m_rotationSetpoint = Rotation2d.kZero;
    private PIDController m_rotationController = new PIDController(DriveConstants.kRotationP, DriveConstants.kRotationI,
            DriveConstants.kRotationD);

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
                .p(DriveConstants.kVelocityP)
                .i(DriveConstants.kVelocityI)
                .d(DriveConstants.kVelocityD);

        // sets max acceleration
        config.closedLoop.maxMotion
                .maxAcceleration(DriveConstants.kMaxAccelerationMetersPerSecondSquared);

        // sets encoder conversion factors
        config.encoder
                .positionConversionFactor(DriveConstants.kRotationsToMeters)
                .velocityConversionFactor(DriveConstants.KRotationsPerMinuteToMetersPerSecond);

        // left side motors
        config.inverted(DriveConstants.kLeftMotorsInverted);

        m_frontLeftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rearLeftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // right side motors
        config.inverted(DriveConstants.kRightMotorsInverted);

        m_frontRightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rearRightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // get encoders
        m_frontLeftEncoder = m_frontLeftMotor.getEncoder();
        m_rearLeftEncoder = m_rearLeftMotor.getEncoder();
        m_frontRightEncoder = m_frontRightMotor.getEncoder();
        m_rearRightEncoder = m_rearRightMotor.getEncoder();

        // get closed loop controllers
        m_frontLeftClosedLoop = m_frontLeftMotor.getClosedLoopController();
        m_rearLeftClosedLoop = m_rearLeftMotor.getClosedLoopController();
        m_frontRightClosedLoop = m_frontRightMotor.getClosedLoopController();
        m_rearRightClosedLoop = m_rearRightMotor.getClosedLoopController();

        m_odometry = new MecanumDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d(),
                getWheelPositions());

        m_driveTab.addNumber("Robot X (m)", () -> m_odometry.getPoseMeters().getX());
        m_driveTab.addNumber("Robot Y (m)", () -> m_odometry.getPoseMeters().getY());
        m_driveTab.addNumber("Robot Heading (deg)", () -> m_odometry.getPoseMeters().getRotation().getDegrees());

        m_driveTab.add("Field", m_field);

        AutoBuilder.configure(m_odometry::getPoseMeters, this::resetOdometry, this::getChassisSpeeds,
                this::drive, AutoConstants.kAutoController, AutoConstants.kRobotConfig, () -> false, this);

        // put drive motors into coast mode when disabled
        RobotModeTriggers.disabled()
                .onTrue(setIdleMode(IdleMode.kCoast))
                .onFalse(setIdleMode(IdleMode.kBrake));

        m_rotationController.enableContinuousInput(0, 2 * Math.PI);
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
        drive(xSpeed, ySpeed, zRotation, true);
    }

    /**
     * Drives the robot using curvature drive.
     * 
     * @param xSpeed        The robot's speed along the X axis [-1, 1].
     *                      Forward is positive.
     * @param ySpeed        The robot's speed along the Y axis [-1, 1].
     *                      Left is positive.
     * @param zRotation     The normalized curvature [-1, 1].
     *                      Counterclockwise is positive.
     * @param fieldRelative Whether or not to use field relative controls. Defaults
     *                      to true.
     */
    public void drive(double xSpeed, double ySpeed, double zRotation, boolean fieldRelative) {
        xSpeed *= DriveConstants.kMaxForwardSpeedMetersPerSecond;
        ySpeed *= DriveConstants.kMaxStrafeSpeedMetersPerSecond;
        zRotation *= DriveConstants.kMaxRotationSpeedRadiansPerSecond;

        // m_rotationSetpoint = m_rotationSetpoint
        //         .plus(Rotation2d.fromRadians(zRotation).times(TimedRobot.kDefaultPeriod));

        // // continuously adjust for potential drift
        // zRotation = m_rotationController.calculate(m_gyro.getRotation2d().getRadians(),
        //         m_rotationSetpoint.getRadians());

        m_prevZRotation = zRotation;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zRotation);

        if (fieldRelative)
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getFieldRelativeHeading());

        drive(chassisSpeeds);
    }

    /**
     * Drives the robot based on raw robot relative {@link ChassisSpeeds}.
     * 
     * @param speeds The speeds to drive the robot with.
     */
    public void drive(ChassisSpeeds speeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(speeds);

        m_frontLeftClosedLoop.setReference(wheelSpeeds.frontLeftMetersPerSecond, ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot0, m_feedforward.calculate(wheelSpeeds.frontLeftMetersPerSecond));

        m_rearLeftClosedLoop.setReference(wheelSpeeds.rearLeftMetersPerSecond, ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot0, m_feedforward.calculate(wheelSpeeds.rearLeftMetersPerSecond));

        m_frontRightClosedLoop.setReference(wheelSpeeds.frontRightMetersPerSecond,
                ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot0, m_feedforward.calculate(wheelSpeeds.frontRightMetersPerSecond));

        m_rearRightClosedLoop.setReference(wheelSpeeds.rearRightMetersPerSecond, ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot0, m_feedforward.calculate(wheelSpeeds.rearRightMetersPerSecond));
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
        return runOnce(this::resetRotationSetpoint).andThen(run(() -> {
            double xSpeed = -xSpeedSupplier.getAsDouble();
            double ySpeed = -ySpeedSupplier.getAsDouble();
            double zRotation = -zRotationSupplier.getAsDouble();

            xSpeed = MathUtil.applyDeadband(
                    ControllerUtils.axisSensitivity(xSpeed, DriveConstants.kXAxisSensitvity), DriveConstants.kDeadband);
            ySpeed = MathUtil.applyDeadband(
                    ControllerUtils.axisSensitivity(ySpeed, DriveConstants.kYAxisSensitvity), DriveConstants.kDeadband);
            zRotation = MathUtil.applyDeadband(
                    ControllerUtils.axisSensitivity(zRotation, DriveConstants.kRotationAxisSensitivity),
                    DriveConstants.kDeadband);

            drive(xSpeed, ySpeed, zRotation);
        }));
    }

    /**
     * Stops the robot.
     */
    public void stopDrive() {
        resetRotationSetpoint();
        drive(0, 0, 0);
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
                m_rearLeftEncoder.getVelocity(),
                m_rearRightEncoder.getVelocity());

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
                m_rearLeftEncoder.getPosition(),
                m_rearRightEncoder.getPosition());
    }

    /**
     * Resets the current tracked robot position to the specified {@link Pose2d}.
     * 
     * @param pose The Pose2d object.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(m_gyro.getRotation2d(), getWheelPositions(), pose);
    }

    /**
     * Resets the current rotation setpoint to the current rotation.
     */
    private void resetRotationSetpoint() {
        m_rotationSetpoint = m_gyro.getRotation2d();
        m_rotationController.reset();
    }

    /**
     * Sets the {@link IdleMode} for all drivetrain motors. This will not persist
     * through power cycles.
     * 
     * @param mode The idle mode to set.
     */
    public void setIdleMode(IdleMode mode) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(mode);

        for (SparkMax motor : new SparkMax[] { m_frontLeftMotor, m_rearLeftMotor, m_frontRightMotor,
                m_rearLeftMotor }) {
            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    /**
     * Creates a {@link Command} that resets the drive train's field relative controls offset.
     * @return The command.
     */
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
