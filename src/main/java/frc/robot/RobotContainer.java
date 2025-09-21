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

    // The robot's subsystem defined here
    private final DriveTrain m_driveTrain = new DriveTrain();

    // controllers
    private final CommandFlightHotasX m_driverController = new CommandFlightHotasX(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_coDriverController = new CommandXboxController(
            OperatorConstants.kCoDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
     * constructor with an arbitrary predicate, or via the named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
     * for {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
     * Flight joysticks}.
     */
    private void configureBindings() {
        m_driveTrain.setDefaultCommand(
                m_driveTrain.driveJoysticks(m_driverController::getStickY, m_driverController::getStickX,
                        m_driverController::getRudder));

        ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator");

        operatorTab.addBoolean("L1", m_driverController.getHID()::getL1Button);
        operatorTab.addBoolean("L2", m_driverController.getHID()::getL2Button);
        operatorTab.addBoolean("L3", m_driverController.getHID()::getL3Button);
        operatorTab.addBoolean("R1", m_driverController.getHID()::getR1Button);
        operatorTab.addBoolean("R2", m_driverController.getHID()::getR2Button);
        operatorTab.addBoolean("R3", m_driverController.getHID()::getR3Button);
        operatorTab.addBoolean("5", m_driverController.getHID()::get5Button);
        operatorTab.addBoolean("6", m_driverController.getHID()::get6Button);
        operatorTab.addBoolean("7", m_driverController.getHID()::get7Button);
        operatorTab.addBoolean("8", m_driverController.getHID()::get8Button);
        operatorTab.addBoolean("Start", m_driverController.getHID()::getStartButton);
        operatorTab.addBoolean("Select", m_driverController.getHID()::getSelectButton);

        operatorTab.addDouble("Stick X", m_driverController::getStickX);
        operatorTab.addDouble("Stick Y", m_driverController::getStickY);
        operatorTab.addDouble("Throttle", m_driverController::getThrottle);
        operatorTab.addDouble("Rudder", m_driverController::getRudder);
        operatorTab.addDouble("Rocker", m_driverController::getRockerAxis);
        operatorTab.addDouble("POV", m_driverController.getHID()::getPOV);
    }
}
