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
    
    private SparkMax elevatorMotor;
    private RelativeEncoder encoder;
    private ProfiledPIDController elevatorPidController;
    private DigitalInput bottomInput, topInput;

    // this might need to be added to other functions or commands
    private boolean isPIDMode = false;

    public Elevator() {

        // sets motor
        elevatorMotor = new SparkMax(ElevatorConstants.kElevatorMotorid, MotorType.kBrushless);
        //sets PID controller
        elevatorPidController = new ProfiledPIDController(
            ElevatorConstants.kElevatorP, 
            ElevatorConstants.kElevatorI, 
            ElevatorConstants.kElevatorD, 
            new Constraints(
                ElevatorConstants.kElevatorMaxSpeedInchesPerSecond,
                ElevatorConstants.kElevatorMaxAccelerationInchesPerSecondSquared));

        // limit switches 
        bottomInput = new DigitalInput(ElevatorConstants.kBottomInputChannel);
        topInput = new DigitalInput(ElevatorConstants.kTopInputChannel);

        //configure
        SparkMaxConfig masterConfig = new SparkMaxConfig();

        masterConfig
        .idleMode(ElevatorConstants.kElevatorIdleMode)
        .smartCurrentLimit(ElevatorConstants.kSmartCurrentLimit)
        .inverted(ElevatorConstants.kInverted);
        
        masterConfig.encoder
        .positionConversionFactor(ElevatorConstants.kElevatorEncoderPositionFactor)
        .velocityConversionFactor(ElevatorConstants.kElevatorEncoderVelocityFactor);

        masterConfig.limitSwitch
        .reverseLimitSwitchEnabled(false)
        .forwardLimitSwitchEnabled(false);

        
        elevatorMotor.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // gets encoder
        encoder = elevatorMotor.getEncoder();


    }

    public void setElevatorHeight(double height) {
        height = MathUtil.clamp(height, 0, ElevatorConstants.kElevatorMaxHeight);
        elevatorPidController.reset(getHeight(), getVelocity()); // idk if this is the same thing as set 
        elevatorPidController.setGoal(height);

        isPIDMode = true;
    }

    /** 
     * this is presets for the elevator
     * @param index
     */
    public void setElevatorLevel(int index) {
        setElevatorHeight(ElevatorConstants.kElevatorLevelHeights[index]);
    }

    public void setVelocity(double velocity) {

        isPIDMode = false;
        
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

        elevatorMotor.set(velocity);

    }

    /**
     * Creates a {@link Command} that moves the elevator to the specified height.
     * @param height The height to move the elevator to in meters.
     * @param endImmediately Whether the command should end immediately or wait until the elevator has reached the height.
     * @return The runnable <code>Command</code>.
     */
    public Command goToHeight(double height, boolean endImmediately) {
        if (height == 0) {
            return runOnce(() -> setElevatorHeight(height))
            .andThen(new WaitUntilCommand(() -> elevatorPidController.atGoal() | endImmediately))
            .andThen(zeroElevator())
            .withName("Go");
        }
        
        return runOnce(() -> setElevatorHeight(height))
            .andThen(new WaitUntilCommand(() -> 
                elevatorPidController.atGoal() || endImmediately
            ))
            .withName("Go");
    }

    /**
     * Creates a {@link Command} that moves the elevator to the specified height.
     * @param height The height to move the elevator to in meters.
     * @return The runnable <code>Command</code>.
     */
    public Command goToHeight(double height) {
        return goToHeight(height, false);
    }

    /**
     * Creates a {@link Command} Moves the elevator down until it touches the magnetic sensor.
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
     * @param velocity The velocity to set the elevator to.
     * @return The runnable <code>Command</code>.
     */
    public Command goToVelocity(double velocity) {
        return runOnce(() -> setVelocity(velocity));
    }

    /**
     * Creates a {@link Command} that moves to elevator to the specified level.
     * @param index The index of the level.
     * @return The runnable <code>Command</code>.
     */
    public Command goToLevel(int index) {
        return goToLevel(index, false);
    }

    /**
     * Creates a {@link Command} that moves to elevator to the specified level.
     * @param index The index of the level.
     * @param endImmediately Whether the command should end immediately or wait until the elevator has reached the level.
     * @return The runnable <code>Command</code>.
     */
    public Command goToLevel(int index, boolean endImmediately) {
        return goToHeight(
            ElevatorConstants.kElevatorLevelHeights[index],
            endImmediately
        );
    }

    /**
     * Creates a {@link Command} that sets the position of the elevator encoder to zero.
     * @return The runnable <code>Command</code>.
     */
    public Command zeroEncoder() {
        return runOnce(() -> encoder.setPosition(0)).ignoringDisable(true);
    }


    public double getVelocity() {
        return encoder.getVelocity();
    }

    public double getHeight() {
        return encoder.getPosition();
    }

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

    public void stopElevator() {
        setElevatorHeight(getHeight());
    }

    public boolean isAtBottom() {
        return bottomInput.get();
    }

    public boolean isAtTop() {
        return topInput.get();
    }

    @Override
    public void periodic() {
        final double currentHeight = getHeight();

        if (isPIDMode) {
            // double velocitySetpoint = m_elevatorPIDController.getSetpoint().velocity;
            
            elevatorMotor.set(
                elevatorPidController.calculate(currentHeight) + ElevatorConstants.kElevatorG
                // m_elevatorFeedforward.calculateWithVelocities(getElevatorVelocity(), velocitySetpoint)
            );
        }
        
        /*
         * TODO: this might not work since I changed it to an encoder to set the postition 
         */

        if (isAtBottom()) {
            // Prevent the elevator from going down when it reaches the bottom
            // by preventing the speed from being negative
            elevatorMotor.set(Math.max(ElevatorConstants.kElevatorG, elevatorMotor.get()));
            encoder.setPosition(0);
        } else if (isAtTop() || currentHeight >= ElevatorConstants.kElevatorMaxHeight) {
            // Prevent the elevator from going up when it reaches the top
            // by preventing the speed from being positive
            elevatorMotor.set(Math.min(ElevatorConstants.kElevatorG, elevatorMotor.get()));
        }
    }
}  


