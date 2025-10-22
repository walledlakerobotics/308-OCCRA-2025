package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/*
 *  A subsystem that controls the intake motor's
 */
public class Intake extends SubsystemBase {

    private SparkMax m_IntakeMotor;
    /*
     * Constructs intake
     */
    public Intake() {
        SparkMaxConfig config = new SparkMaxConfig();

        m_IntakeMotor = new SparkMax(IntakeConstants.kLeaderMotorId, MotorType.kBrushless);

        config
                .smartCurrentLimit(IntakeConstants.kSmartCurrentLimit)
                .idleMode(IntakeConstants.kIdleMode)
                .inverted(IntakeConstants.kLeaderMotorInverted);

        m_IntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the speed of the two motors.
     * 
     * @param speed The speed to set.
     */
    public void setSpeed(double speed) {
        m_IntakeMotor.set(speed);
    }

    /**
     * Returns a command to intake.
     * 
     * @return
     */
    public Command intake() {
        return runOnce(() -> setSpeed(IntakeConstants.kIntakeSpeed))
                .andThen(Commands.waitSeconds(IntakeConstants.kClawTime))
                .finallyDo(() -> setSpeed(0));
    }

    /**
     * Returns a command to outtake.
     * 
     * @return
     */
    public Command outtake() {
        return runOnce(() -> setSpeed(-IntakeConstants.kOuttakeSpeed))
                .andThen(Commands.waitSeconds(IntakeConstants.kClawTime))
                .finallyDo(() -> setSpeed(0));
    }

}
