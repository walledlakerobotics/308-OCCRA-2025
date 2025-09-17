// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.CommandFlightHotasX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Json defined parser
    private final JSONParser parser = new JSONParser();

    // The robot's subsystem defined here
    private final DriveTrain m_driveSubsystem = new DriveTrain();

    // controllers
    private final CommandFlightHotasX m_driverController = new CommandFlightHotasX(0);
    private final CommandXboxController m_coDriverController = new CommandXboxController(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_driveSubsystem.setDefaultCommand(
                m_driveSubsystem.driveJoysticks(m_driverController::getThrottle, m_driverController::getStickX,
                        m_driverController.getHID()::getL1Button));

        configureBindings(OperatorConstants.kDriverJsonPath, OperatorConstants.kCoDriverJsonPath, 1, 1);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings(String driverPath, String coDriverPath, int driverSetting, int coDriverSetting) {
        File mainDriverBindsJson = new File(driverPath);
        File coDriverBindsJson = new File(coDriverPath);

        ShuffleboardTab test = Shuffleboard.getTab("Test");

        test.addBoolean("L1", m_driverController.getHID()::getL1Button);
        test.addBoolean("L2", m_driverController.getHID()::getL2Button);
        test.addBoolean("L3", m_driverController.getHID()::getL3Button);
        test.addBoolean("R1", m_driverController.getHID()::getR1Button);
        test.addBoolean("R2", m_driverController.getHID()::getR2Button);
        test.addBoolean("R3", m_driverController.getHID()::getR3Button);
        test.addBoolean("5", m_driverController.getHID()::get5Button);
        test.addBoolean("6", m_driverController.getHID()::get6Button);
        test.addBoolean("7", m_driverController.getHID()::get7Button);
        test.addBoolean("8", m_driverController.getHID()::get8Button);
        test.addBoolean("Start", m_driverController.getHID()::getStartButton);
        test.addBoolean("Select", m_driverController.getHID()::getSelectButton);

        test.addDouble("Stick X", m_driverController::getStickX);
        test.addDouble("Stick Y", m_driverController::getStickY);
        test.addDouble("Throttle", m_driverController::getThrottle);
        test.addDouble("Rudder", m_driverController::getRudder);
        test.addDouble("Rocker", m_driverController::getRockerAxis);
        test.addDouble("POV", m_driverController.getHID()::getPOV);

        try {
            JSONObject driverFileData = (JSONObject) parser.parse(new FileReader(mainDriverBindsJson));
            JSONObject coDriverFileData = (JSONObject) parser.parse(new FileReader(coDriverBindsJson));

            JSONObject driverData = (JSONObject) driverFileData.get("mainDriver " + driverSetting);
            JSONObject coDriverData = (JSONObject) coDriverFileData.get("coDriver " + coDriverSetting);

        } catch (IOException | ParseException e) {

        }

    }
}
