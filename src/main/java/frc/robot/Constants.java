// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    private Constants() {
        throw new UnsupportedOperationException("This is a constants class!");
    }

    public static class OperatorConstants {
        private OperatorConstants() {
            throw new UnsupportedOperationException("This is a constants class!");
        }

        // ports for the controllers
        public static final int kDriverControllerPort = 0;
        public static final int kCoDriverControllerPort = 1;
    }

    public static class DriveConstants {
        private DriveConstants() {
            throw new UnsupportedOperationException("This is a constants class!");
        }

        // ids for the motors
        public static final int kFrontLeftMotorId = 3;
        public static final int kFrontRightMotorId = 5;
        public static final int kBackLeftMotorId = 2;
        public static final int kBackRightMotorId = 4;

        // sets if an motor is inverted
        public static final boolean kLeftMotorsInverted = false;
        public static final boolean kRightMotorsInverted = true;

        // idle mode
        public static final IdleMode kMotorIdleMode = IdleMode.kBrake;

        // smartLimit
        public static final int kSmartCurrentLimit = 30;

        public static final double kMaxSpeedMetersPerSecond = 5.6;

        // PID constants for controlling wheel velocity
        public static final double kVelocityP = 0.1;
        public static final double kVelocityI = 0.0;
        public static final double kVelocityD = 0.0;
        public static final double kVelocityFF = 1 / 473;

        // physical constants
        public static final double kWheelRadius = Units.inchesToMeters(3);
        public static final double kWheelDiameter = 2 * kWheelRadius;
        public static final double kWheelCircumference = kWheelDiameter * Math.PI;
        public static final double kGearRatio = 8.45865;

        public static final double kTrackWidth = 0.5;
        public static final double kWheelBase = 0.5;

        // encoder conversion factors
        public static final double kRotationsToMeters = kWheelCircumference / kGearRatio;
        public static final double KRotationsPerMinuteToMetersPerSecond = kRotationsToMeters / 60;

        // kinematics
        public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(
                // Front left
                new edu.wpi.first.math.geometry.Translation2d(kWheelBase / 2, kTrackWidth / 2),
                // Front right
                new edu.wpi.first.math.geometry.Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                // Back left
                new edu.wpi.first.math.geometry.Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                // Back right
                new edu.wpi.first.math.geometry.Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // this makes it so if there any stick drift it prevents the robot from move due
        // to the stick drift
        public static final double kDeadBand = 0.01;

        public static final double kXAxisSensitvity = 0.4;
        public static final double kYAxisSensitvity = 0.4;
        public static final double kRotationAxisSensitivity = 0.4;

        public static final double kXAxisMultiplier = 1;
        public static final double kYAxisMultiplier = 1;
        public static final double kRotationAxisMultiplier = 0.7;
    }

    public static class AutoConstants {
        private AutoConstants() {
            throw new UnsupportedOperationException("This is a constants class!");
        }

        public static final PIDConstants kTranslationConstants = new PIDConstants(1, 0, 0);
        public static final PIDConstants kRotationConstants = new PIDConstants(1, 0, 0);

        public static final double kMassKG = 50;
        public static final double kRobotMOI = 50;

        public static final double kWheelCOF = 1;

        public static final DCMotor kDriveMotor = DCMotor.getNEO(1);

        public static final RobotConfig kRobotConfig = new RobotConfig(kMassKG, kRobotMOI,
                new ModuleConfig(DriveConstants.kWheelDiameter / 2, DriveConstants.kMaxSpeedMetersPerSecond,
                        kWheelCOF, kDriveMotor, DriveConstants.kSmartCurrentLimit, 1),
                DriveConstants.kTrackWidth);

        public static final PathFollowingController kAutoController = new PPHolonomicDriveController(
                kTranslationConstants, kRotationConstants);
    }

    public static class ElevatorConstants {
        private ElevatorConstants() {
            throw new UnsupportedOperationException("This is a constants class!");
        }

        // ( UwU ) its the id of the elevator motor
        public static final int kElevatorLeaderMotorId = 0;
        public static final int kElevatorFollowerMotorId = 0;

        // sets if motor is inverted
        public static final boolean kLeaderMotorInverted = false;
        public static final boolean kFollowerMotorInverted = false;
        // level heights this will change
        public static final double[] kElevatorLevelHeights = { 0, 7.5, 14.5, 23.55 };
        // idlemode
        public static final IdleMode kElevatorIdleMode = IdleMode.kBrake;
        // max height of elevator
        public static final double kElevatorMaxHeight = 25;
        /** The P for the elevator PID. */
        public static final double kElevatorP = 1.3;
        /** The I for the elevator PID. */
        public static final double kElevatorI = 0;
        /** The D for the elevator PID. */
        public static final double kElevatorD = 0;
        /** The gravity feed forward for the elevator. */
        public static final double kElevatorG = 0;
        // current limit
        public static final int kSmartCurrentLimit = 60;

        /** The reduction in distance calculated by endcoders due to gear ratio. */
        public static final double kElevatorReduction = 20;
        /** The diameter of the gear/wheel that moves the elevator in inches. */
        public static final double kGearDiameter = 1;
        /** The circumference of the gear/wheel that moves the elevator. */
        public static final double kGearCircumference = kGearDiameter * Math.PI;
        /**
         * The conversion factor that converts from motor rotations to inches travelled.
         */
        public static final double kElevatorEncoderPositionFactor = kGearCircumference / kElevatorReduction;
        /**
         * The conversion factor that converts from motor rotations per minute to inches
         * travelled per second.
         */
        public static final double kElevatorEncoderVelocityFactor = (kGearCircumference / kElevatorReduction) / 60;

        /** The maximum speed the elevator can move at with full power. */
        public static final double kElevatorFreeSpeedMetersPerSecond = NEOMotorConstants.kFreeSpeedRpm
                * kElevatorEncoderVelocityFactor;

        /** The maximum allowed speed the elevator should move at. */
        public static final double kElevatorMaxSpeedInchesPerSecond = kElevatorFreeSpeedMetersPerSecond;

        /** The maximum allowed acceleration of the elevator. */
        public static final double kElevatorMaxAccelerationInchesPerSecondSquared = 30;

        /** The manual movement speed of the elevator. */
        public static final double kElevatorManualSpeed = 0.5;

        // limit switch channels
        public static final int kTopInputChannel = 0;
        public static final int kBottomInputChannel = 0;
    }

    public static final class NEOMotorConstants {
        private NEOMotorConstants() {
            throw new UnsupportedOperationException("This is a constants class!");
        }

        /** The maximum speed the motors go run at in revolutions per minute. */
        public static final double kFreeSpeedRpm = 5676;
        /** The maximum speed the motors go run at in revolutions per second. */
        public static final double kFreeSpeedRps = kFreeSpeedRpm / 60;
    }

    /**
     * Describe how the {@link Arm} should rotate the coral arm.
     */
    public static final class ArmConstants {
        private ArmConstants() {
            throw new UnsupportedOperationException("This is a constants class!");
        }

        /** The CAN ID of the arm motor. */
        public static final int kArmMotorCanId = 22;
        /** The smart current limit for the motor */
        public static final int kSmartCurrentLimit = 30;
        /** The idle mode of the motor. */
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        /** Whether to invert the direction of the arm motor. */
        public static final boolean kArmMotorInverted = false;

        /** The reduction causes by the gear ratio of the motor. */
        public static final double kGearReduction = 1;
        /** The position conversion factor of the arm encoder. */
        public static final double kPositionEncoderConversionFactor = 1 / kGearReduction;
        /** The velocity conversion factor of the arm encoder. */
        public static final double kVelocityEncoderConversionFactor = 1 / kGearReduction / 60;
        /** The maximum speed of the arm in rotations per second. */
        public static final double kArmMaxSpeedRPS = 0.5;
        /** The maximum acceleration of the arm in rotations per second squared. */
        public static final double kArmMaxAccelerationRPSSquared = 1;

        /** The angle offset for the motor encoder such that when the encoder returns 0 the arm is parallel to the floor. */
        public static final Rotation2d kEncoderAngleOffset = Rotation2d.fromDegrees(0);

        /** The P for the arm PID controller. */
        public static final double kArmP = 2.5;
        /** The I for the arm PID controller. */
        public static final double kArmI = 0;
        /** The D for the arm PID controller. */
        public static final double kArmD = 0;
        
        /** The S gain for the arm feedforward. */
        public static final double kArmS = 0;
        /** The gravity gain for the arm feedforward. */
        public static final double kArmG = 0.02;
        /** The V gain for the arm feedforward. */
        public static final double kArmV = 0;
        /** The A gain for the arm feedforward. */
        public static final double kArmA = 0;

        /** The speed to move the arm at while intaking. */
        public static final double kArmIntakingSpeed = 0.1;

        /** The angles of the arm for each reef level. */
        public static final Rotation2d[] kArmLevelAngles = {
            Rotation2d.kZero,
            Rotation2d.fromDegrees(32.4),
            Rotation2d.fromDegrees(32.4),
            Rotation2d.fromDegrees(32.4)
        };
    }

    public static final class IntakeConstants {

        private IntakeConstants() {
            throw new UnsupportedOperationException("This is a constants class!");
        }

        //ids of the motors
        public static final int kLeaderMotorId = 0;
        public static final int kFollowerMotorId = 0;

        //sets if the motors are inverted
        public static final boolean kLeaderMotorInverted = false;
        public static final boolean kFollowerMotorInverted = false;
        
        //current limit of the motors
        public static final int kSmartCurrentLimit = 0;
        /** The idle mode of the motor. */
        public static final IdleMode kIdleMode = IdleMode.kBrake;

    }
}