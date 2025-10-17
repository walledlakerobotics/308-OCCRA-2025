package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private SparkMax m_elevatorLeader, m_elevatorFollower;
    private RelativeEncoder m_elevatorEncoder;
    private SparkClosedLoopController m_elevatorPIDController;
    private DigitalInput m_bottomLimit, m_topLimit;

    private Double m_elevatorGoal = null;

    public Elevator() {
        // sets motors
        m_elevatorLeader = new SparkMax(ElevatorConstants.kElevatorLeaderMotorId, MotorType.kBrushless);
        m_elevatorFollower = new SparkMax(ElevatorConstants.kElevatorFollowerMotorId, MotorType.kBrushless);

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
                .positionConversionFactor(ElevatorConstants.kElevatorRotationsToMeters)
                .velocityConversionFactor(ElevatorConstants.kElevatorRotationsPerMinuteToMetersPerSecond);

        config.closedLoop
                .p(ElevatorConstants.kElevatorP)
                .i(ElevatorConstants.kElevatorI)
                .d(ElevatorConstants.kElevatorD);

        config.closedLoop.maxMotion
                .maxVelocity(ElevatorConstants.kElevatorMaxSpeedMetersPerSecond)
                .maxVelocity(ElevatorConstants.kElevatorMaxAccelerationMetersPerSecondSquared);

        m_elevatorLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config
                .follow(m_elevatorLeader)
                .inverted(ElevatorConstants.kFollowerMotorInverted);

        m_elevatorFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // gets encoder
        m_elevatorEncoder = m_elevatorLeader.getEncoder();

        // gets closed loop
        m_elevatorPIDController = m_elevatorLeader.getClosedLoopController();
    }

    /**
     * Gets the current height of the elevator in meters.
     * 
     * @return The height of the elevator.
     */
    public void setHeight(double height) {
        height = MathUtil.clamp(height, 0, ElevatorConstants.kTopSwitchHeight);
        m_elevatorGoal = height;

        m_elevatorPIDController.setReference(height, ControlType.kPosition, ClosedLoopSlot.kSlot0,
                ElevatorConstants.kElevatorG);
    }

    /**
     * Moves the elevator to the height of the specified level index.
     * 
     * @param index
     */
    public void setLevel(int index) {
        setHeight(ElevatorConstants.kElevatorLevelHeights[index]);
    }

    /**
     * Sets the velocity of the elevator.
     * 
     * @param velocity The velocity to set the motor to in meters per second.
     */
    public void setVelocity(double velocity) {
        m_elevatorGoal = null;

        if (isAtBottom()) {
            // Prevent the elevator from going down when it reaches the bottom
            // by preventing the speed from being negative
            velocity = Math.max(0, velocity);
        } else if (isAtTop()) {
            // Prevent the elevator from going down when it reaches the top
            // by preventing the speed from being positive
            velocity = Math.min(0, velocity);
        }

        m_elevatorPIDController.setReference(velocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0,
                ElevatorConstants.kElevatorG);

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
                .andThen(new WaitUntilCommand(() -> atGoal() || endImmediately))
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
     * Returns the current velocity of the elevator in meters per second.
     * 
     * @return The double value.
     */
    public double getVelocity() {
        return m_elevatorEncoder.getVelocity();
    }

    /**
     * Gets the current height of the elevator.
     * 
     * @return The double value.
     */
    public double getHeight() {
        return m_elevatorEncoder.getPosition();
    }

    /**
     * Gets whether or not the elevator has reached its height goal.
     * @return The boolean value.
     */
    public boolean atGoal() {
        return Math.abs(getHeight() - m_elevatorGoal) <= ElevatorConstants.kErrorTolerance;
    }

    /**
     * Gets the current level index of the elevator.
     * 
     * @return The integer index.
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
     * Stops the elevator
     */
    public void stopElevator() {
        setHeight(getHeight());
    }

    /**
     * Gets whether or not the elevator is at the bottom.
     * 
     * @return True if the elevator is at the bottom, or otherwise false.
     */
    public boolean isAtBottom() {
        return m_bottomLimit.get();
    }

    /**
     * Gets whether or not the elevator is at the top.
     * 
     * @return True if the elevator is at the top, or otherwise false.
     */
    public boolean isAtTop() {
        return m_topLimit.get();
    }

    @Override
    public void periodic() {
        if (isAtBottom()) {
            // Prevent the elevator from going down when it reaches the bottom
            // by preventing the speed from being negative
            m_elevatorLeader.set(Math.max(ElevatorConstants.kElevatorG, m_elevatorLeader.get()));
            m_elevatorEncoder.setPosition(0);
        } else if (isAtTop()) {
            // Prevent the elevator from going up when it reaches the top
            // by preventing the speed from being positive
            m_elevatorLeader.set(Math.min(ElevatorConstants.kElevatorG, m_elevatorLeader.get()));
            m_elevatorEncoder.setPosition(ElevatorConstants.kTopSwitchHeight);
        }
    }
}
