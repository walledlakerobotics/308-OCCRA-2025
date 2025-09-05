// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Consumer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import pabeles.concurrency.ConcurrencyOps.Reset;

/*
 * This is still a WIP
 */

public class DriveSubsystem extends SubsystemBase {

  public enum DriveMotor {
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
  }

  // this is where I make the lists of devices like motors and encoders, and
  // configs for the motors
  private SparkMax[] motors = new SparkMax[4];
  private RelativeEncoder[] encoders = new RelativeEncoder[4];

  private ShuffleboardTab driveTab = Shuffleboard.getTab(getName());

  public DriveSubsystem() {
    DriveMotor[] motorPoses = DriveMotor.values();

    // this creates the objects and stores them in array
    for (int i = 0; i < motorPoses.length; i++) {
      motors[i] = new SparkMax(motorPoses[i].getId(), MotorType.kBrushless);
      encoders[i] = motors[i].getEncoder();
    }

    for (RelativeEncoder relativeEncoder : encoders) {
      relativeEncoder.setPosition(0); // this will change, and need to figureout how to get velocity
    }

    configureAllMotors(config -> {
      // sets the idle mode and the smart current limit
      config.smartCurrentLimit(DriveConstants.kSmartLimit);
      config.idleMode(DriveConstants.kMotorIdleMode);

      // sets the PID
      config.closedLoop.pid(DriveConstants.kVelocityP, DriveConstants.kVelocityI, DriveConstants.kVelocityD);
      config.closedLoop.velocityFF(DriveConstants.kVelocityFF);
    }, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // sets if an motor is inverted
    for (DriveMotor motorPose : DriveMotor.values()) {
      configureMotor(motorPose, config -> config.inverted(motorPose.getInverted()), ResetMode.kNoResetSafeParameters,
          PersistMode.kPersistParameters);
    }
  }

  /**
   * 
   * @param speeds
   */
  public void drive(ChassisSpeeds speeds) {
    speeds = new ChassisSpeeds(
        -speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        -speeds.omegaRadiansPerSecond);

    DifferentialDriveWheelSpeeds wheelSpeeds;

    // write speeds to motors
    // setConfig(Constants.DriveConstants.kFrontRightMotorId, config -> config.);
  }

  /**
   * 
   * @param forwardSpeed
   * @param turningSpeed
   */
  public void drive(double forwardSpeed, double turningSpeed) {
    double leftSpeed = forwardSpeed + turningSpeed;
    double rightSpeed = forwardSpeed - turningSpeed;
    double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

    if (max > 1) {
      leftSpeed /= max;
      rightSpeed /= max;
    }

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
  public void setAllSpeed(double speed) {
    applyAllMotors(motor -> motor.set(speed));
  }

  /**
   * 
   * @param motorPose
   * @param function
   */
  private SparkMax getMotor(DriveMotor motorPose) {
    return motors[motorPose.getIndex()];
  }

  /**
   * 
   * @param function
   */
  private void applyAllMotors(Consumer<SparkMax> function) {
    for (SparkMax motor : motors) {
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

  /**
   * 
   * @param configure
   */
  private void configureMotor(DriveMotor motorPose, Consumer<SparkMaxConfig> configure, ResetMode resetMode,
      PersistMode persist) {
    SparkMax motor = getMotor(motorPose);
    SparkMaxConfig config = new SparkMaxConfig();
    configure.accept(config);

    motor.configure(config, resetMode, persist);
  }

  /**
   * 
   * @param motorPose
   * @return
   */
  public double getSpeed(DriveMotor motorPose) {
    return encoders[motorPose.getIndex()].getVelocity();
  }

  /**
   * 
   * @param motorPose
   * @return
   */
  private RelativeEncoder getEncoder(int motorPose) {
    return encoders[motorPose];
  }

  @Override
  public void periodic() {
    // updates
  }

}
