package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    private SparkMax m_elevatorLeader, m_elevatorFollower;
    private RelativeEncoder m_elevatorEncoder;
    private ProfiledPIDController m_elevatorPIDController;
    private DigitalInput m_bottomLimit, m_topLimit;

    private boolean m_isPIDMode = false;

    public Elevator() {
        // sets motors
        m_elevatorLeader = new SparkMax(ElevatorConstants.kElevatorLeaderMotorId, MotorType.kBrushless);
        m_elevatorFollower = new SparkMax(ElevatorConstants.kElevatorFollowerMotorId, MotorType.kBrushless);

        // sets PID controller
        m_elevatorPIDController = new ProfiledPIDController(
                ElevatorConstants.kElevatorP,
                ElevatorConstants.kElevatorI,
                ElevatorConstants.kElevatorD,
                new Constraints(
                        ElevatorConstants.kElevatorMaxSpeedInchesPerSecond,
                        ElevatorConstants.kElevatorMaxAccelerationInchesPerSecondSquared));

        // limit switches
        m_bottomLimit = new DigitalInput(ElevatorConstants.kBottomInputChannel);
        m_topLimit = new DigitalInput(ElevatorConstants.kTopInputChannel);

        // configure
        SparkMaxConfig config = new SparkMaxConfig();

        config
                .idleMode(ElevatorConstants.kElevatorIdleMode)
                .smartCurrentLimit(ElevatorConstants.kSmartCurrentLimit)
                .inverted(ElevatorConstants.kLeaderMotorInverted);

        config.encoder
                .positionConversionFactor(ElevatorConstants.kElevatorEncoderPositionFactor)
                .velocityConversionFactor(ElevatorConstants.kElevatorEncoderVelocityFactor);

        m_elevatorLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config
                .follow(m_elevatorLeader)
                .inverted(ElevatorConstants.kFollowerMotorInverted);

        m_elevatorFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // gets encoder
        m_elevatorEncoder = m_elevatorLeader.getEncoder();
    }

    /**
     * Gets the current height of the elevator in meters.
     * 
     * @return The height of the elevator.
     */
    public void setElevatorHeight(double height) {
        height = MathUtil.clamp(height, 0, ElevatorConstants.kElevatorMaxHeight);
        m_elevatorPIDController.reset(getHeight(), getVelocity());
        m_elevatorPIDController.setGoal(height);

        m_isPIDMode = true;
    }

    /**
     * this is presets for the elevator
     * 
     * @param index
     */
    public void setElevatorLevel(int index) {
        setElevatorHeight(ElevatorConstants.kElevatorLevelHeights[index]);
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
        } else if (isAtTop() || getHeight() >= ElevatorConstants.kElevatorMaxHeight) {
            // Prevent the elevator from going doupwn when it reaches the top
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
        if (height == 0) {
            return runOnce(() -> setElevatorHeight(height))
                    .andThen(new WaitUntilCommand(() -> m_elevatorPIDController.atGoal() | endImmediately))
                    .andThen(zeroElevator())
                    .withName("Go");
        }

        return runOnce(() -> setElevatorHeight(height))
                .andThen(new WaitUntilCommand(() -> m_elevatorPIDController.atGoal() || endImmediately))
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
                .andThen(new WaitUntilCommand(() -> isAtBottom()))
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

    /**
     * returns the current preset levels in the elevator constants
     * 
     * @return
     */
    public int getCurrentLevel() {
        double height = getHeight();
        Double smallestDifference = null;
        int index = 0;

        for (int i = 0; i < ElevatorConstants.kElevatorLevelHeights.length; i++) {
            double difference = Math.abs(ElevatorConstants.kElevatorLevelHeights[i] - height);

            if (smallestDifference == null || difference < smallestDifference) {
                smallestDifference = difference; // Sets new smallest difference if it is
                index = i; // Sets level index to i because it has a smaller difference
            }
        }

        return index;
    }

    /*
     * stops the elevator
     */
    public void stopElevator() {
        setElevatorHeight(getHeight());
    }

    /**
     * returns if the limit switchs are triggered
     * 
     * @return returns if its false or true
     */
    public boolean isAtBottom() {
        return m_bottomLimit.get();
    }

    /**
     * returns if the limit switchs are triggered
     * 
     * @return returns if its false or true
     */
    public boolean isAtTop() {
        return m_topLimit.get();
    }

    @Override
    public void periodic() {
        final double currentHeight = getHeight();

        if (m_isPIDMode) {
            // double velocitySetpoint = m_elevatorPIDController.getSetpoint().velocity;
            m_elevatorLeader.set(
                    m_elevatorPIDController.calculate(currentHeight) + ElevatorConstants.kElevatorG
            // m_elevatorFeedforward.calculateWithVelocities(getElevatorVelocity(),
            // velocitySetpoint)
            );
        }

        if (isAtBottom()) {
            // Prevent the elevator from going down when it reaches the bottom
            // by preventing the speed from being negative
            m_elevatorLeader.set(Math.max(ElevatorConstants.kElevatorG, m_elevatorLeader.get()));
            m_elevatorEncoder.setPosition(0);
        } else if (isAtTop() || currentHeight >= ElevatorConstants.kElevatorMaxHeight) {
            // Prevent the elevator from going up when it reaches the top
            // by preventing the speed from being positive
            m_elevatorLeader.set(Math.min(ElevatorConstants.kElevatorG, m_elevatorLeader.get()));
        }
    }
}
