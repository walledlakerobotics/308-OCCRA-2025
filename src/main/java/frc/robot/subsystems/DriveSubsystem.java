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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

/*
 * This is still a WIP
 */

public class DriveSubsystem extends SubsystemBase {

  //this is where I make the lists of devices like motors and encoders, and configs for the motors
  private ArrayList<SparkMax> sparkMaxs = new ArrayList<>();
  private ArrayList<RelativeEncoder> encoders = new ArrayList<>();
  private ArrayList<SparkMaxConfig> configs = new ArrayList<>();
  
  //sets an array for motor id's
  private int[] idArray = { 
    DriveConstants.kFrontLeftMotorId, 
    DriveConstants.kFrontRightMotorId, 
    DriveConstants.kBackRightMotorId, 
    DriveConstants.kBackLeftMotorId
  };

  private ShuffleboardTab driveTab = Shuffleboard.getTab(getName());

  public DriveSubsystem() {

    // this creates the objects and stores them in array
    for (int i = 0; i < idArray.length; i++) {
      configs.add(new SparkMaxConfig());
      sparkMaxs.add(new SparkMax(idArray[i], MotorType.kBrushless));
      sparkMaxs.get(i).configure(configs.get(i), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      encoders.add(sparkMaxs.get(i).getEncoder());
    }

    for (RelativeEncoder relativeEncoder : encoders) {
      relativeEncoder.setPosition(0); // this will change, and need to figureout how to get velocity

    }

    applyAllConfigs(config -> {
      // sets the idle mode and the smart current limit
      config.smartCurrentLimit(DriveConstants.kSmartLimit);
      config.idleMode(DriveConstants.kMotorIdleMode);

      //sets the PID 
      config.closedLoop.pid(DriveConstants.kVelocityP, DriveConstants.kVelocityI, DriveConstants.kVelocityD);
      config.closedLoop.velocityFF(DriveConstants.kVelocityFF);
    });

    //sets if an motor is inverted
    setConfig(DriveConstants.kFrontRightMotorId, config -> config.inverted(DriveConstants.kFrontRightMotorInverted));
    setConfig(DriveConstants.kFrontLeftMotorId, config -> config.inverted(DriveConstants.kFrontLeftMotorInverted));
    setConfig(DriveConstants.kBackRightMotorId, config -> config.inverted(DriveConstants.kBackRightMotorInverted));
    setConfig(DriveConstants.kBackLeftMotorId, config -> config.inverted(DriveConstants.kBackLeftMotorInverted));

    

  }
  
  /**
   * 
   * @param speeds
   */
  public void drive(ChassisSpeeds speeds) {
    speeds = new ChassisSpeeds(
      -speeds.vxMetersPerSecond, 
      speeds.vyMetersPerSecond, 
      -speeds.omegaRadiansPerSecond
    );

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

  //stops all the motors
  public void stopDrive() {
    applyAllMotors(motor -> motor.set(0));
  }
  
  //sets all idlemodes to be brake
  public void setBrakeMode() {
    applyAllConfigs(config -> config.idleMode(IdleMode.kBrake));
  }

  //sets all idlemodes to be coast
  public void setCoastMode() {
    applyAllConfigs(config -> config.idleMode(IdleMode.kCoast));
  }

  /**
   * This will take the speed param and set all the motor's to that speed
   * @param speed
   */
  public void setAllSpeed(double speed) {
    applyAllMotors(motor -> motor.set(speed));
  }

  /**
   * @param motorId
   * @param function
   */
  public void setConfig(int motorId, Consumer<SparkMaxConfig> function) {
    for (int i = 0; i < sparkMaxs.size(); i++) {
      if (sparkMaxs.get(i).getDeviceId() == motorId) {
        function.accept(configs.get(i));
      }
    }
  }

  /**
   * 
   * @param motorId
   * @param function
   */
  public void setMotor(int motorId, Consumer<SparkMax> function) {
    for (int i = 0; i < sparkMaxs.size(); i++) {
      if (sparkMaxs.get(i).getDeviceId() == motorId) {
        function.accept(sparkMaxs.get(i));
      }
    }
  }

  /**
   * 
   * @param function
   */
  public void applyAllMotors(Consumer<SparkMax> function) {
    for (int i = 0; i < sparkMaxs.size(); i++) {
      function.accept(sparkMaxs.get(i));
    }
  }

  /**
   * 
   * @param function
   */
  public void applyAllConfigs(Consumer<SparkMaxConfig> function) { 
    for (int i = 0; i < configs.size(); i++) {
      function.accept(configs.get(i));
    }
  }

  /**
   * 
   * @param id
   * @return
   */
  public double getSpeed(int id) {
    for (SparkMax motor : sparkMaxs) {
      if (motor.getDeviceId() == id) {
        return motor.get();
      }
    }

    return 0;
  }

  /**
   * 
   * @param motorId
   * @return
   */
  public RelativeEncoder getEncoder(int motorId) {
    for (int i = 0; i < sparkMaxs.size(); i++) {
      if (sparkMaxs.get(i).getDeviceId() == motorId) {
        return encoders.get(i);
      }
    }
    return null;
  }

  @Override
  public void periodic() {
    //updates 
  }

}
