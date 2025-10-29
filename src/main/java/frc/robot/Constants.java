// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Arm;
import frc.robot.utils.CANIDs;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * 
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
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
        public static final int kFrontLeftMotorId = CANIDs.frontLeft();
        public static final int kRearLeftMotorId = CANIDs.rearLeft();
        public static final int kFrontRightMotorId = CANIDs.frontRight();
        public static final int kRearRightMotorId = CANIDs.rearRight();

        // sets if an motor is inverted
        public static final boolean kLeftMotorsInverted = false;
        public static final boolean kRightMotorsInverted = true;

        // idle mode
        public static final IdleMode kMotorIdleMode = IdleMode.kBrake;

        // smart current limit
        public static final int kSmartCurrentLimitAmps = 30;

        public static final double kMaxForwardSpeedMetersPerSecond = 5.6;
        public static final double kMaxStrafeSpeedMetersPerSecond = 5.6;
        public static final double kMaxRotationSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5.6;

        // PID constants for controlling wheel velocity
        public static final double kVelocityP = 0.1;
        public static final double kVelocityI = 0.0;
        public static final double kVelocityD = 0.0;
        public static final double kVelocityS = 0.0;
        public static final double kVelocityV = 0.0;
        public static final double kVelocityA = 0.0;

        // PID constants for controlling robot rotation
        public static final double kRotationP = 7.0;
        public static final double kRotationI = 0.0;
        public static final double kRotationD = 0.0;

        // physical constants
        public static final double kWheelRadiusMeters = Units.inchesToMeters(3);
        public static final double kWheelDiameterMeters = 2 * kWheelRadiusMeters;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kGearReduction = 8.45865;

        public static final double kTrackWidthMeters = Units.inchesToMeters(23.5);
        public static final double kWheelBaseMeters = Units.inchesToMeters(20.5);

        // encoder conversion factors
        public static final double kRotationsToMeters = kWheelCircumferenceMeters / kGearReduction;
        public static final double KRotationsPerMinuteToMetersPerSecond = kRotationsToMeters / 60;

        // kinematics
        public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(
                // Front left
                new edu.wpi.first.math.geometry.Translation2d(kWheelBaseMeters / 2, kTrackWidthMeters / 2),
                // Front right
                new edu.wpi.first.math.geometry.Translation2d(kWheelBaseMeters / 2, -kTrackWidthMeters / 2),
                // Back left
                new edu.wpi.first.math.geometry.Translation2d(-kWheelBaseMeters / 2, kTrackWidthMeters / 2),
                // Back right
                new edu.wpi.first.math.geometry.Translation2d(-kWheelBaseMeters / 2, -kTrackWidthMeters / 2));

        // this makes it so if there any stick drift it prevents the robot from move due
        // to the stick drift
        public static final double kDeadband = 0.01;

        public static final double kXAxisSensitvity = 0.9;
        public static final double kYAxisSensitvity = 0.9;
        public static final double kRotationAxisSensitivity = 0.9;
    }

    public static class AutoConstants {
        private AutoConstants() {
            throw new UnsupportedOperationException("This is a constants class!");
        }

        public static final PIDConstants kTranslationConstants = new PIDConstants(1, 0, 0);
        public static final PIDConstants kRotationConstants = new PIDConstants(1, 0, 0);

        public static final RobotConfig kRobotConfig;

        static {
            try {
                kRobotConfig = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }

        public static final PathFollowingController kAutoController = new PPHolonomicDriveController(
                kTranslationConstants, kRotationConstants);
    }

    public static class ElevatorConstants {
        private ElevatorConstants() {
            throw new UnsupportedOperationException("This is a constants class!");
        }

        // ( UwU ) its the id of the elevator motor
        public static final int kElevatorLeaderMotorId = CANIDs.secondaryMotor(0);
        public static final int kElevatorFollowerMotorId = 0;

        // sets if motor is inverted
        public static final boolean kLeaderMotorInverted = true;
        public static final boolean kFollowerMotorInverted = false;
        // level heights this will change
        public static final double[] kElevatorLevelHeights = { 0 };
        // idlemode
        public static final IdleMode kElevatorIdleMode = IdleMode.kBrake;
        // max height of elevator
        public static final double kTopSwitchHeight = 0.5;
        /** The P for the elevator PID. */
        public static final double kElevatorP = 1.3;
        /** The I for the elevator PID. */
        public static final double kElevatorI = 0;
        /** The D for the elevator PID. */
        public static final double kElevatorD = 0;
        /** The gravity feed forward for the elevator. */
        public static final double kElevatorG = 0;
        // current limit
        public static final int kSmartCurrentLimit = 30;

        public static final double kErrorTolerance = 0.05;

        /** The reduction in distance calculated by endcoders due to gear ratio. */
        public static final double kElevatorReduction = 20;
        /** The diameter of the gear/wheel that moves the elevator in meters. */
        public static final double kGearDiameter = Units.inchesToMeters(1);
        /** The circumference of the gear/wheel that moves the elevator. */
        public static final double kGearCircumference = kGearDiameter * Math.PI;

        /**
         * The conversion factor that converts from motor rotations to meters travelled.
         */
        public static final double kElevatorRotationsToMeters = kGearCircumference / kElevatorReduction;
        /**
         * The conversion factor that converts from motor rotations per minute to meters
         * travelled per second.
         */
        public static final double kElevatorRotationsPerMinuteToMetersPerSecond = kElevatorRotationsToMeters / 60;

        /** The maximum allowed speed the elevator should move at. */
        public static final double kElevatorMaxSpeedMetersPerSecond = 0.5;

        /** The maximum allowed acceleration of the elevator. */
        public static final double kElevatorMaxAccelerationMetersPerSecondSquared = 0.5;

        /** The manual movement speed of the elevator. */
        public static final double kElevatorManualSpeed = 0.5;

        // limit switch channels
        public static final int kTopInputChannel = 0;
        public static final int kBottomInputChannel = 0;
    }

    /**
     * Describe how the {@link Arm} should rotate the arm.
     */
    public static final class ArmConstants {
        private ArmConstants() {
            throw new UnsupportedOperationException("This is a constants class!");
        }

        /** The CAN ID of the arm motor. */
        public static final int kArmMotorCanId = CANIDs.secondaryMotor(2);
        /** The smart current limit for the motor */
        public static final int kSmartCurrentLimit = 30;
        /** The idle mode of the motor. */
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        /** Whether to invert the direction of the arm motor. */
        public static final boolean kArmMotorInverted = false;

        /** The maximum speed of the arm in rotations per second. */
        public static final double kArmMaxSpeedRPS = 0.5;
        /** The maximum acceleration of the arm in rotations per second squared. */
        public static final double kArmMaxAccelerationRPSSquared = 1;

        /**
         * The angle offset for the motor encoder such that when the encoder returns 0
         * the arm is parallel to the floor.
         */
        public static final Rotation2d kEncoderAngleOffset = Rotation2d.fromDegrees(0);

        /** The P for the arm PID controller. */
        public static final double kArmP = 2.5;
        /** The I for the arm PID controller. */
        public static final double kArmI = 0;
        /** The D for the arm PID controller. */
        public static final double kArmD = 0;

        /** The gravity gain for the arm feedforward. */
        public static final double kArmG = 0.02;

        /** The speed to move the arm at while intaking. */
        public static final double kArmIntakingSpeed = 0.1;
    }

    public static final class ClawConstants {
        private ClawConstants() {
            throw new UnsupportedOperationException("This is a constants class!");
        }

        // ids of the motors
        public static final int kClawMotorId = CANIDs.secondaryMotor(1);

        // whether to invert the motors
        public static final boolean kLeaderMotorInverted = false;
        public static final boolean kFollowerMotorInverted = false;

        // current limit of the motors
        public static final int kSmartCurrentLimit = 30;

        // idle mode of the motors
        public static final IdleMode kIdleMode = IdleMode.kBrake;

        // speeds when intaking and outtaking
        public static final double kClawSpeed = 1;

        public static final double kClawTime = 1;

        public static final int kCloseInputChannel = 0;
        public static final int kOpenInputChannel = 0;
    }
}