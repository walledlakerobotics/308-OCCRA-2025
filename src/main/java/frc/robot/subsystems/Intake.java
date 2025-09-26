package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CubeIntakeConstants;

public class Intake extends SubsystemBase {
    
    private SparkMax m_leaderMotor, m_followerMotor;
    private RelativeEncoder m_Encoder;

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
            .inverted(CubeIntakeConstants.kFollowerMotorInverted)
            .follow(m_leaderMotor.getDeviceId());
        
        m_followerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {

    }


}
