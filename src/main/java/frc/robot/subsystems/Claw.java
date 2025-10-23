package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

/*
 *  A subsystem that controls the intake motor's
 */
public class Claw extends SubsystemBase {

    private SparkMax m_IntakeMotor;
    private DigitalInput m_CloseSwitch, m_OpenSwitch;
    /*
     * Constructs intake
     */
    public Claw() {
        SparkMaxConfig config = new SparkMaxConfig();

        m_CloseSwitch = new DigitalInput(ClawConstants.kCloseInputChannel);
        m_OpenSwitch = new DigitalInput(ClawConstants.kOpenInputChannel);

        m_IntakeMotor = new SparkMax(ClawConstants.kLeaderMotorId, MotorType.kBrushless);

        config
                .smartCurrentLimit(ClawConstants.kSmartCurrentLimit)
                .idleMode(ClawConstants.kIdleMode)
                .inverted(ClawConstants.kLeaderMotorInverted);

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
        return runOnce(() -> setSpeed(ClawConstants.kIntakeSpeed))
                .andThen(Commands.waitSeconds(ClawConstants.kClawTime))
                .finallyDo(() -> setSpeed(0));
    }

    /**
     * Returns a command to outtake.
     * 
     * @return
     */
    public Command outtake() {
        return runOnce(() -> setSpeed(-ClawConstants.kOuttakeSpeed))
                .andThen(Commands.waitSeconds(ClawConstants.kClawTime))
                .finallyDo(() -> setSpeed(0));
    }

    public boolean isClawClosed() {
        return m_CloseSwitch.get();
    }

    public boolean isClawOpen() {
        return m_OpenSwitch.get();
    }

}
