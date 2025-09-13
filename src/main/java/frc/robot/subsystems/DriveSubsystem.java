// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/*
 * This is still a WIP
 */

public class DriveSubsystem extends SubsystemBase {

    public enum DriveMotorPose {
        kFrontLeft, kFrontRight, kBackRight, kBackLeft;

        public int getId() {
            return switch (this) {
                case kFrontLeft -> DriveConstants.kFrontLeftMotorId;
                case kFrontRight -> DriveConstants.kFrontRightMotorId;
                case kBackRight -> DriveConstants.kBackRightMotorId;
                case kBackLeft -> DriveConstants.kBackLeftMotorId;
            };
        }

        public int getIndex() {
            return switch (this) {
                case kFrontLeft -> 0;
                case kFrontRight -> 1;
                case kBackRight -> 2;
                case kBackLeft -> 3;
            };
        }

        public boolean getInverted() {
            return switch (this) {
                case kFrontLeft -> DriveConstants.kFrontLeftMotorInverted;
                case kFrontRight -> DriveConstants.kFrontRightMotorInverted;
                case kBackRight -> DriveConstants.kBackRightMotorInverted;
                case kBackLeft -> DriveConstants.kBackLeftMotorInverted;
            };
        }

        public static DriveMotorPose fromIndex(int index) {
            return switch (index) {
                case 0 -> DriveMotorPose.kFrontLeft;
                case 1 -> DriveMotorPose.kFrontRight;
                case 2 -> DriveMotorPose.kBackRight;
                case 3 -> DriveMotorPose.kBackLeft;
                default -> null;
            };
        }
    }

    // this is where I make the lists of devices like motors and encoders, and
    // configs for the motors
    private final SparkMax[] m_motors = new SparkMax[4];
    private final RelativeEncoder[] m_encoders = new RelativeEncoder[4];
    private final SparkClosedLoopController[] m_closesLoopControllers = new SparkClosedLoopController[4];

    // private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

    // private final DifferentialDriveOdometry m_odometry = new
    // DifferentialDriveOdometry(
    // m_gyro.getRotation2d(), getLeftPosition(), getRightPosition());

    private ShuffleboardTab m_driveTab = Shuffleboard.getTab(getName());

    public DriveSubsystem() {
        // this creates the objects and stores them in array
        for (int i = 0; i < 4; i++) {
            DriveMotorPose motorPose = DriveMotorPose.fromIndex(i);

            m_motors[i] = new SparkMax(motorPose.getId(), MotorType.kBrushless);

            SparkMaxConfig config = new SparkMaxConfig();

            // sets the idle mode, the smart current limit, and the inversion
            config.smartCurrentLimit(DriveConstants.kSmartLimit);
            config.idleMode(DriveConstants.kMotorIdleMode);
            config.inverted(motorPose.getInverted());

            // sets the PID
            config.closedLoop.pid(DriveConstants.kVelocityP, DriveConstants.kVelocityI, DriveConstants.kVelocityD);
            config.closedLoop.velocityFF(DriveConstants.kVelocityFF);

            // sets encoder conversion factors
            config.encoder.positionConversionFactor(DriveConstants.kRotationsToMeters);
            config.encoder.velocityConversionFactor(DriveConstants.KRotationsPerMinuteToMetersPerSecond);

            // applies the config to the motor
            m_motors[i].configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            m_encoders[i] = m_motors[i].getEncoder();
            m_closesLoopControllers[i] = m_motors[i].getClosedLoopController();

            // AutoBuilder.configure(m_odometry::getPoseMeters, this::resetOdometry,
            // this::getChassisSpeeds, this::drive, new DifferentialDrive(ZZ, null),
            // null, null, null);
        }
    }

    /**
     * 
     * @param speeds
     */
    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(speeds);

        // write speeds to motors
        getClosedLoop(DriveMotorPose.kFrontLeft).setReference(wheelSpeeds.leftMetersPerSecond, ControlType.kVelocity);
        getClosedLoop(DriveMotorPose.kBackLeft).setReference(wheelSpeeds.leftMetersPerSecond, ControlType.kVelocity);
        getClosedLoop(DriveMotorPose.kFrontRight).setReference(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity);
        getClosedLoop(DriveMotorPose.kBackRight).setReference(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity);
    }

    /**
     * 
     * @param forwardSpeed
     * @param turningSpeed
     */
    public void drive(double forwardSpeed, double turningSpeed) {
        forwardSpeed *= DriveConstants.kMaxForwardSpeed;
        turningSpeed *= DriveConstants.kMaxRotationSpeed;

        double leftSpeed = forwardSpeed + turningSpeed;
        double rightSpeed = forwardSpeed - turningSpeed;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

        if (max > 1) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        getClosedLoop(DriveMotorPose.kFrontLeft)
                .setReference(DriveConstants.kMaxPhysicalSpeedMetersPerSecond * leftSpeed, ControlType.kVelocity);
        getClosedLoop(DriveMotorPose.kBackLeft)
                .setReference(DriveConstants.kMaxPhysicalSpeedMetersPerSecond * leftSpeed, ControlType.kVelocity);
        getClosedLoop(DriveMotorPose.kFrontRight)
                .setReference(DriveConstants.kMaxPhysicalSpeedMetersPerSecond * rightSpeed, ControlType.kVelocity);
        getClosedLoop(DriveMotorPose.kBackRight)
                .setReference(DriveConstants.kMaxPhysicalSpeedMetersPerSecond * rightSpeed, ControlType.kVelocity);
    }

    // stops all the motors
    public void stopDrive() {
        applyAllMotors(motor -> motor.set(0));
    }

    // sets all idlemodes to be brake
    public void setBrakeMode() {
        configureAllMotors(config -> config.idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    // sets all idlemodes to be coast
    public void setCoastMode() {
        configureAllMotors(config -> config.idleMode(IdleMode.kCoast), ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    /**
     * This will take the speed param and set all the motor's to that speed
     * 
     * @param speed
     */
    public void setAllMotorSpeeds(double speed) {
        applyAllMotors(motor -> motor.set(speed));
    }

    /**
     * 
     * @param motorPose
     * @param function
     */
    private SparkMax getMotor(DriveMotorPose motorPose) {
        return m_motors[motorPose.getIndex()];
    }

    /**
     * 
     * @param function
     */
    private void applyAllMotors(Consumer<SparkMax> function) {
        for (SparkMax motor : m_motors) {
            function.accept(motor);
        }
    }

    /**
     * 
     * @param configure
     */
    private void configureAllMotors(Consumer<SparkMaxConfig> configure, ResetMode resetMode, PersistMode persist) {
        applyAllMotors(motor -> {
            SparkMaxConfig config = new SparkMaxConfig();
            configure.accept(config);

            motor.configure(config, resetMode, persist);
        });
    }

    private ChassisSpeeds getChassisSpeeds() {
        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
        ChassisSpeeds speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(wheelSpeeds);
        // speeds.omegaRadiansPerSecond = Units.degreesToRadians(m_gyro.getRate());

        return speeds;
    }

    /**
     * 
     * @return
     */
    public double getLeftPosition() {
        return getEncoder(DriveMotorPose.kFrontLeft).getPosition()
                + getEncoder(DriveMotorPose.kBackLeft).getPosition() / 2;
    }

    /**
     * 
     * @return
     */
    public double getLeftSpeed() {
        return getEncoder(DriveMotorPose.kFrontLeft).getVelocity()
                + getEncoder(DriveMotorPose.kBackLeft).getVelocity() / 2;
    }

    /**
     * 
     * @return
     */
    public double getRightPosition() {
        return getEncoder(DriveMotorPose.kFrontRight).getPosition()
                + getEncoder(DriveMotorPose.kBackRight).getPosition();
    }

    /**
     * 
     * @return
     */
    public double getRightSpeed() {
        return getEncoder(DriveMotorPose.kFrontRight).getVelocity()
                + getEncoder(DriveMotorPose.kBackRight).getVelocity() / 2;
    }

    /**
     * 
     * @param motorPose
     * @return
     */
    private RelativeEncoder getEncoder(DriveMotorPose motorPose) {
        return m_encoders[motorPose.getIndex()];
    }

    /**
     * 
     * @param motorPose
     * @return
     */
    private SparkClosedLoopController getClosedLoop(DriveMotorPose motorPose) {
        return m_closesLoopControllers[motorPose.getIndex()];
    }

    public void resetOdometry(Pose2d pose) {
        // m_odometry.resetPosition(m_gyro.getRotation2d(), getLeftPosition(),
        // getRightPosition(), pose);
    }

    @Override
    public void periodic() {
        // m_odometry.update(m_gyro.getRotation2d(), getLeftPosition(),
        // getRightPosition());
    }
}
