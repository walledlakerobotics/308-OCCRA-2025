package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private SparkMax m_elevatorLeader;
    private RelativeEncoder m_elevatorEncoder;
    private ProfiledPIDController m_elevatorPIDController;
    private SparkLimitSwitch m_bottomLimit;

    private final ShuffleboardTab m_elevatorTab = Shuffleboard.getTab("Elevator");

    private boolean m_isPIDMode = false;

    public Elevator() {
        // sets motors
        m_elevatorLeader = new SparkMax(ElevatorConstants.kElevatorLeaderMotorId, MotorType.kBrushless);
        // m_elevatorFollower = new SparkMax(ElevatorConstants.kElevatorFollowerMotorId, MotorType.kBrushless);

        // sets PID controller
        m_elevatorPIDController = new ProfiledPIDController(
                ElevatorConstants.kElevatorP,
                ElevatorConstants.kElevatorI,
                ElevatorConstants.kElevatorD,
                new Constraints(
                        ElevatorConstants.kElevatorMaxSpeedMetersPerSecond,
                        ElevatorConstants.kElevatorMaxAccelerationMetersPerSecondSquared));

        // limit switches
        m_bottomLimit = m_elevatorLeader.getReverseLimitSwitch();
        // m_topLimit = new DigitalInput(ElevatorConstants.kTopInputChannel);

        // configure
        SparkMaxConfig config = new SparkMaxConfig();

        config
                .idleMode(ElevatorConstants.kElevatorIdleMode)
                .smartCurrentLimit(ElevatorConstants.kSmartCurrentLimit)
                .inverted(ElevatorConstants.kLeaderMotorInverted);

        config.encoder
                .positionConversionFactor(ElevatorConstants.kElevatorRotationsToMeters)
                .velocityConversionFactor(ElevatorConstants.kElevatorRotationsPerMinuteToMetersPerSecond);

        m_elevatorLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config
                .follow(m_elevatorLeader)
                .inverted(ElevatorConstants.kFollowerMotorInverted);

        // m_elevatorFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // gets encoder
        m_elevatorEncoder = m_elevatorLeader.getEncoder();

        m_elevatorTab.addDouble("Elevator Height", this::getHeight);
    }

    /**
     * Gets the current height of the elevator in meters.
     * 
     * @return The height of the elevator.
     */
    public void setHeight(double height) {
        height = MathUtil.clamp(height, 0, ElevatorConstants.kTopSwitchHeight);
        m_elevatorPIDController.reset(getHeight(), getVelocity());
        m_elevatorPIDController.setGoal(height);

        m_isPIDMode = true;
    }

    /**
     * Moves the elevator to the height of the specified level index.
     * 
     * @param index The index of the level.
     */
    public void setLevel(int index) {
        setHeight(ElevatorConstants.kElevatorLevelHeights[index]);
    }

    /**
     * Sets the velocity of the elevator.
     * 
     * @param velocity The velocity to set the motor to from <code>-1</code> to
     *                 <code>1</code>.
     */
    public void setVelocity(double velocity) {

        m_isPIDMode = false;

        velocity += ElevatorConstants.kElevatorG;

        if (isAtBottom()) {
            // Prevent the elevator from going down when it reaches the bottom
            // by preventing the speed from being negative
            velocity = Math.max(0, velocity);
        } else if (isAtTop()) {
            // Prevent the elevator from going down when it reaches the top
            // by preventing the speed from being positive
            velocity = Math.min(0, velocity);
        }

        m_elevatorLeader.set(velocity);

    }

    /**
     * Creates a {@link Command} that moves the elevator to the specified height.
     * 
     * @param height         The height to move the elevator to in meters.
     * @param endImmediately Whether the command should end immediately or wait
     *                       until the elevator has reached the height.
     * @return The runnable <code>Command</code>.
     */
    public Command goToHeight(double height, boolean endImmediately) {
        return runOnce(() -> setHeight(height))
                .andThen(Commands.waitUntil(() -> m_elevatorPIDController.atGoal() || endImmediately))
                .andThen(height == 0 ? zeroElevator() : Commands.none())
                .withName("Go");
    }

    /**
     * Creates a {@link Command} that moves the elevator to the specified height.
     * 
     * @param height The height to move the elevator to in meters.
     * @return The runnable <code>Command</code>.
     */
    public Command goToHeight(double height) {
        return goToHeight(height, false);
    }

    /**
     * Creates a {@link Command} Moves the elevator down until it touches the
     * magnetic sensor.
     * 
     * @returns The runnable <code>Command</code>
     */
    public Command zeroElevator() {
        return goToVelocity(-ElevatorConstants.kElevatorManualSpeed)
                .andThen(Commands.waitUntil(() -> isAtBottom()))
                .finallyDo(() -> stopElevator())
                .withTimeout(0.5);
    }

    /**
     * Creates a {@link Command} that sets the velocity of the elevator.
     * 
     * @param velocity The velocity to set the elevator to.
     * @return The runnable <code>Command</code>.
     */
    public Command goToVelocity(double velocity) {
        return runOnce(() -> setVelocity(velocity));
    }

    /**
     * Creates a {@link Command} that moves to elevator to the specified level.
     * 
     * @param index The index of the level.
     * @return The runnable <code>Command</code>.
     */
    public Command goToLevel(int index) {
        return goToLevel(index, false);
    }

    /**
     * Creates a {@link Command} that moves to elevator to the specified level.
     * 
     * @param index          The index of the level.
     * @param endImmediately Whether the command should end immediately or wait
     *                       until the elevator has reached the level.
     * @return The runnable <code>Command</code>.
     */
    public Command goToLevel(int index, boolean endImmediately) {
        return goToHeight(
                ElevatorConstants.kElevatorLevelHeights[index],
                endImmediately);
    }

    /**
     * Creates a {@link Command} that sets the position of the elevator encoder to
     * zero.
     * 
     * @return The runnable <code>Command</code>.
     */
    public Command zeroEncoder() {
        return runOnce(() -> m_elevatorEncoder.setPosition(0)).ignoringDisable(true);
    }

    /**
     * returns the encoder's velocity.
     * 
     * @return
     */
    public double getVelocity() {
        return m_elevatorEncoder.getVelocity();
    }

    /**
     * returns the encoders position -> which is the current height of the elevator.
     * 
     * @return
     */
    public double getHeight() {
        return m_elevatorEncoder.getPosition();
    }

    /*
     * stops the elevator
     */
    public void stopElevator() {
        setHeight(getHeight());
    }

    /**
     * returns if the limit switchs are triggered
     * 
     * @return returns if its false or true
     */
    public boolean isAtBottom() {
        return m_bottomLimit.isPressed();
    }

    /**
     * returns if the limit switchs are triggered
     * 
     * @return returns if its false or true
     */
    public boolean isAtTop() {
        // return m_topLimit.get();
        return false;
    }

    @Override
    public void periodic() {
        final double currentHeight = getHeight();

        if (m_isPIDMode) {
            m_elevatorLeader.set(
                    -m_elevatorPIDController.calculate(currentHeight) + ElevatorConstants.kElevatorG
            );
        }

        if (isAtBottom()) {
            m_elevatorEncoder.setPosition(0);
        } else if (isAtTop()) {
            m_elevatorEncoder.setPosition(ElevatorConstants.kTopSwitchHeight);
        }
    }
}
