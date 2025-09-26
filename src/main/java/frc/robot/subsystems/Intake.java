package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/*
 *  A subsystem that controls the intake motor's
 */
public class Intake extends SubsystemBase {
    
    private SparkMax m_leaderMotor, m_followerMotor;
    private RelativeEncoder m_Encoder;

    /*
     * Constructs intake
     */
    public Intake() {
        SparkMaxConfig config = new SparkMaxConfig();

        m_leaderMotor = new SparkMax(IntakeConstants.kLeaderMotorId, MotorType.kBrushless);
        m_followerMotor = new SparkMax(IntakeConstants.kFollowerMotorId, MotorType.kBrushless);

        m_Encoder = m_leaderMotor.getEncoder();

        config
            .smartCurrentLimit(IntakeConstants.kSmartCurrentLimit)
            .idleMode(IntakeConstants.kIdleMode)
            .inverted(IntakeConstants.kLeaderMotorInverted);
        
        m_leaderMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config
            .inverted(IntakeConstants.kFollowerMotorInverted)
            .follow(m_leaderMotor.getDeviceId());
        
        m_followerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the speed of the two motors.
     * @param speed <- passes the speed into the motor.
     */
    public void setSpeed(double speed) {
        m_leaderMotor.set(speed);
    }
    
    /**
     * Returns a command to intake.
     * @return
     */
    public Command intake() {
        return runEnd(
            () -> setSpeed(0), 
            () -> setSpeed(0));
    }

    /**
     * Returns a command to outtake.
     * @return
     */
    public Command deploy() {
        return runEnd(
            () -> setSpeed(0),
            () -> setSpeed(0));
    }
}
