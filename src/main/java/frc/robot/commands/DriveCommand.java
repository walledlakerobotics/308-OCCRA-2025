package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {

    private DriveSubsystem m_DriveSubsystem;
    private DoubleSupplier leftJoyStick, rightJoyStick;
    private BooleanSupplier turboMode;

    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier leftInput, DoubleSupplier rightInput, BooleanSupplier turboMode) {
        this.m_DriveSubsystem = driveSubsystem;
        this.leftJoyStick = leftInput;
        this.rightJoyStick = rightInput;
        this.turboMode = turboMode;

        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void execute() {
        m_DriveSubsystem.drive(0, 0);;
    }

    @Override
    public void end(boolean interrupted) {
        m_DriveSubsystem.stopDrive();
    }
    
}
