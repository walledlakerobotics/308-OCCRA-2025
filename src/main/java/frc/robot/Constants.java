// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static class OperatorConstants {
    private OperatorConstants() {
      throw new UnsupportedOperationException("This is a utility class!");
    }

    // Ports for the controllers, and json file relative path
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
    public static final String kDriverJsonPath = "src/main/java/frc/robot/assets/data/mainDriverBinds/mainDriver.json";
    public static final String kCoDriverJsonPath = "src/main/java/frc/robot/assets/data/coDriverBinds/coDriver.json";
  }

  public static class DriveConstants {
    private DriveConstants() {
      throw new UnsupportedOperationException("This is a utility class!");
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
    public static final double kWheelDiameter = Units.inchesToMeters(6);
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;
    public static final double kGearRatio = 8.45865;

    public static final double kTrackWidth = 0.5;

    // encoder conversion factors
    public static final double kRotationsToMeters = kWheelCircumference / kGearRatio;
    public static final double KRotationsPerMinuteToMetersPerSecond = kRotationsToMeters / 60;

    // kinematics
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);

    // this makes it so if there any stick drift it prevents the robot from move due
    // to the stick drift
    public static final double kDeadBand = 0.01;

    public static final double kForwardSensitvity = 0.4;
    public static final double kRotatonSenitvity = 0.4;

    public static final double kMaxForwardPercent = 1;
    public static final double kMaxTurningPercent = 0.7;

  }

  public static class MotorConstants {
    private MotorConstants() {
      throw new UnsupportedOperationException("This is a utility class!");
    }

    public static final int kNeoMotorMaxRPM = 5676;
  }
}
