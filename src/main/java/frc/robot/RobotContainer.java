// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

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
    private final Elevator m_elevator = new Elevator();
    private final Arm m_arm = new Arm();
    private final Claw m_claw = new Claw();

    // controllers
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_coDriverController = new CommandXboxController(
            OperatorConstants.kCoDriverControllerPort);

    private SendableChooser<Command> m_autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();

        m_autoChooser = AutoBuilder.buildAutoChooser();

        Shuffleboard.getTab("Autonomous").add("Auto", m_autoChooser);
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
                m_driveTrain.drive(m_driverController::getLeftY, m_driverController::getLeftX,
                        m_driverController::getRightX));

        m_driverController.a()
                .onTrue(m_driveTrain.resetFieldRelative());

        m_coDriverController.povUp()
                .onTrue(m_elevator.goToVelocity(ElevatorConstants.kElevatorManualSpeed))
                .onFalse(m_elevator.goToVelocity(0));
        m_coDriverController.povDown()
                .onTrue(m_elevator.goToVelocity(-ElevatorConstants.kElevatorManualSpeed))
                .onFalse(m_elevator.goToVelocity(0));

        m_coDriverController.leftBumper()
                .onTrue(m_claw.goToVelocity(ClawConstants.kClawSpeed))
                .onFalse(m_claw.goToVelocity(0));

        m_coDriverController.rightBumper()
                .onTrue(m_claw.goToVelocity(-ClawConstants.kClawSpeed))
                .onFalse(m_claw.goToVelocity(0));
    }

    /**
     * Gets the {@link Command} to run in autonomous.
     */
    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
