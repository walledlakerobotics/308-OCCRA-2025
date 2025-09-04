// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {

    //Ports for the controllers, and json file relitive path
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 0;
    public static final String kDriverJsonPath = "src/main/java/frc/robot/assets/data/mainDriverBinds/mainDriver.json";
    public static final String kCoDriverJsonPath = "src/main/java/frc/robot/assets/data/coDriverBinds/coDriver.json";
  }

  public static class DriveConstants {

    // id's for the motors
    public static final int kFrontLeftMotorId = 0;
    public static final int kFrontRightMotorId = 1;
    public static final int kBackLeftMotorId = 2;
    public static final int kBackRightMotorId = 3;

    // sets if an motor is inverted
    public static final boolean kFrontLeftMotorInverted = false;
    public static final boolean kFrontRightMotorInverted = false;
    public static final boolean kBackLeftMotorInverted = false;
    public static final boolean kBackRightMotorInverted = false;

    // idle mode
    public static final IdleMode kMotorIdleMode = IdleMode.kBrake;

    // smartLimit
    public static final int kSmartLimit = 30;

    // public static final double kMaxSpeedMetersPerSecond = 
    //   ((MotorConstants.kNeoMotorMaxRPM / 60.0) / kGearRatio) * kWheelCircumference;
    public static final double kMaxSpeedMetersPerSecond = 5.6;

    // PID constants for controlling wheel velocity
    public static final double kVelocityP = 0.1;
    public static final double kVelocityI = 0.0;
    public static final double kVelocityD = 0.0;
    public static final double kVelocityFF = 1 / kMaxSpeedMetersPerSecond;

    // this makes it so if there any stick drift it prevents the robot from move due to the stick drift
    public static final double kDeadBand = 0;    

    public static final double kDriverSensitvity = 0;
    public static final double kRotatonSenitvity = 0;

    public static final double kMaxForwardSpeed = 0;
    public static final double kMaxRotationSpeed = 0;
    
  }
}
