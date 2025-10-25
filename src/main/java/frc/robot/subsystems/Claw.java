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

    private SparkMax m_intakeMotor;
    private DigitalInput m_closeSwitch, m_openSwitch;
    /*
     * Constructs intake
     */
    public Claw() {
        SparkMaxConfig config = new SparkMaxConfig();

        m_closeSwitch = new DigitalInput(ClawConstants.kCloseInputChannel);
        m_openSwitch = new DigitalInput(ClawConstants.kOpenInputChannel);

        m_intakeMotor = new SparkMax(ClawConstants.kLeaderMotorId, MotorType.kBrushless);

        config
                .smartCurrentLimit(ClawConstants.kSmartCurrentLimit)
                .idleMode(ClawConstants.kIdleMode)
                .inverted(ClawConstants.kLeaderMotorInverted);

        m_intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the speed of the two motors.
     * 
     * @param speed The speed to set.
     */
    public void setSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    /**
     * Returns a command to intake.
     * 
     * @return
     */
    public Command open() {
        return runOnce(() -> setSpeed(ClawConstants.kIntakeSpeed))
                .andThen(Commands.waitUntil(this::isClawOpen))
                .finallyDo(() -> setSpeed(0));
    }

    /**
     * Returns a command to outtake.
     * 
     * @return
     */
    public Command close() {
        return runOnce(() -> setSpeed(-ClawConstants.kOuttakeSpeed))
                .andThen(Commands.waitUntil(this::isClawClosed))
                .finallyDo(() -> setSpeed(0));
    }

    public boolean isClawClosed() {
        return m_closeSwitch.get();
    }

    public boolean isClawOpen() {
        return m_openSwitch.get();
    }

}
