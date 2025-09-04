// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.FHXController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  //Json defined parser 
  private final JSONParser parser = new JSONParser();

  // The robot's subsystem defined here
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  //controllers
  private final FHXController m_driverController = new FHXController(0);
  private final CommandXboxController m_coDriverController = new CommandXboxController(1);

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings(OperatorConstants.kDriverJsonPath, OperatorConstants.kCoDriverJsonPath, 1, 1);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings(String driverPath, String coDriverPath, int driverSetting, int coDriverSetting) {
    File mainDriverBindsJson = new File(driverPath);
    File coDriverBindsJson = new File(coDriverPath);

    try {
      JSONObject driverFileData = (JSONObject) parser.parse(new FileReader(mainDriverBindsJson));
      JSONObject coDriverFileData = (JSONObject) parser.parse(new FileReader(coDriverBindsJson));

      JSONObject driverData = (JSONObject) driverFileData.get("mainDriver " + driverSetting);
      JSONObject coDriverData = (JSONObject) coDriverFileData.get("coDriver " + coDriverSetting);

      

    } catch (IOException | ParseException e) {
      
    }

    

  }
}
