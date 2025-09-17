// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

/**
 * Handle input from Xbox controllers connected to the Driver Station.
 *
 * <p>
 * This class handles Flight Hotas X input that comes from the Driver Station.
 * Each time a value is requested the most recent value is returned. There is a
 * single class instance for each controller and the mapping of ports to
 * hardware buttons depends on the code in the Driver Station.
 */
public class FlightHotasX extends GenericHID {
    /** Represents a digital button on a FlightHotasX. */
    public enum Button {
        /** R1 button. */
        kR1(1),
        /** R2 button. */
        kR2(9),
        /** R3 button. */
        kR3(3),
        /** L1 button. */
        kL1(2),
        /** L2 button. */
        kL2(10),
        /** L3 button. */
        kL3(4),
        /** 5 button. */
        k5(5),
        /** 6 button. */
        k6(6),
        /** 7 button. */
        k7(7),
        /** 8 button. */
        k8(8),
        /** Start button. */
        kStart(12),
        /** Select button. */
        kSelect(11);

        /** Button value. */
        public final int value;

        Button(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the button, matching the relevant methods.
         * This is done by stripping the leading `k`, and appending `Button`.
         *
         * <p>
         * Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the button.
         */
        @Override
        public String toString() {
            // Remove leading `k`
            return this.name().substring(1) + "Button";
        }
    }

    /** Represents an axis on a FlightHotasX. */
    public enum Axis {
        /** Joystsick X axis. */
        kStickX(0),
        /** Joystick Y axis. */
        kStickY(1),
        /** Throttle axis */
        kThrottle(2),
        /** Rudder axis. */
        kRudder(3),
        /** Rocker axis. */
        kRocker(4);

        /** Axis value. */
        public final int value;

        Axis(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the axis, matching the relevant methods. This
         * is done by stripping the leading `k`, and appending `Axis` if the name ends
         * with `Trigger`.
         *
         * <p>
         * Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the axis.
         */
        @Override
        public String toString() {
            var name = this.name().substring(1); // Remove leading `k`
            if (name.endsWith("Trigger")) {
                return name + "Axis";
            }
            return name;
        }
    }

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is
     *             plugged into (0-5).
     */
    public FlightHotasX(int port) {
        super(port);
        HAL.report(tResourceType.kResourceType_Controller, port + 1);
    }

    /**
     * Get the X axis value of the joystick of the controller. Right is positive.
     * 
     * @return The axis value.
     */
    public double getStickX() {
        return getRawAxis(Axis.kStickX.value);
    }

    /**
     * Get the Y axis value of the joystick of the controller. Back is positive.
     * 
     * @return The axis value.
     */
    public double getStickY() {
        return getRawAxis(Axis.kStickY.value);
    }

    /**
     * Get the throttle axis value of the controller. Forward is positive.
     * 
     * @return The axis value.
     */
    public double getThrottle() {
        return -getRawAxis(Axis.kThrottle.value);
    }

    /**
     * Get the rudder axis value of the controller. Right is positive.
     * 
     * @return The axis value.
     */
    public double getRudder() {
        return getRawAxis(Axis.kRudder.value);
    }

    /**
     * Get the rocker axis value of the controller. Right is positive.
     * 
     * @return The axis value.
     */
    public double getRockerAxis() {
        return getRawAxis(Axis.kRocker.value);
    }

    /**
     * Constructs an event instance around the axis value of the left rocker. The
     * returned trigger will be true when the axis value is less than
     * {@code -threshold}.
     * 
     * @param loop      the event loop instance to attach the event to.
     * @param threshold the negative of the maximum axis value for the returned
     *                  {@link BooleanEvent} to be true. This value should be in the
     *                  range [0, 1] where 0 is the unpressed state of the axis.
     * @return an event instance that is true when the rockers's axis is below
     *         the negative of the provided threshold, attached to the given event
     *         loop.
     */
    public BooleanEvent getLeftRockerTrigger(double threshold, EventLoop loop) {
        return axisLessThan(Axis.kRocker.value, -threshold, loop);
    }

    /**
     * Constructs an event instance around the axis value of the rocker. The
     * returned trigger will be true when the axis value is less than -0.5.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the rockers's axis is below
     *         the negative of the provided threshold, attached to the given event
     *         loop.
     */
    public BooleanEvent getLeftRockerTrigger(EventLoop loop) {
        return getLeftRockerTrigger(0.5, loop);
    }

    /**
     * Constructs an event instance around the axis value of the rocker. The
     * returned trigger will be true when the axis value is greater than
     * {@code threshold}.
     * 
     * @param loop      the event loop instance to attach the event to.
     * @param threshold the minimum axis value for the returned {@link BooleanEvent}
     *                  to be true. This value should be in the range [0, 1] where 0
     *                  is the unpressed state of the axis.
     * @return an event instance that is true when the rockers's axis exceeds
     *         the provided threshold, attached to the given event loop.
     */
    public BooleanEvent getRightRockerTrigger(double threshold, EventLoop loop) {
        return axisGreaterThan(Axis.kRocker.value, threshold, loop);
    }

    /**
     * Constructs an event instance around the axis value of the rocker. The
     * returned trigger will be true when the axis value is greater than
     * 0.5.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the rockers's axis exceeds
     *         the provided threshold, attached to the given event loop.
     */
    public BooleanEvent getRightRockerTrigger(EventLoop loop) {
        return getLeftRockerTrigger(0.5, loop);
    }

    /**
     * Read the value of the R1 button on the controller.
     * 
     * @return The state of the button.
     */
    public boolean getR1Button() {
        return getRawButton(Button.kR1.value);
    }

    /**
     * Whether the R1 button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean getR1ButtonPressed() {
        return getRawButtonPressed(Button.kR1.value);
    }

    /**
     * Whether the R1 button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean getR1ButtonReleased() {
        return getRawButtonReleased(Button.kR1.value);
    }

    /**
     * Constructs an event instance around the R1 button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the R1 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent R1(EventLoop loop) {
        return button(Button.kR1.value, loop);
    }

    /**
     * Read the value of the R2 button on the controller.
     * 
     * @return The state of the button.
     */
    public boolean getR2Button() {
        return getRawButton(Button.kR2.value);
    }

    /**
     * Whether the R2 button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean getR2ButtonPressed() {
        return getRawButtonPressed(Button.kR2.value);
    }

    /**
     * Whether the R2 button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean getR2ButtonReleased() {
        return getRawButtonReleased(Button.kR2.value);
    }

    /**
     * Constructs an event instance around the R2 button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the R2 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent R2(EventLoop loop) {
        return button(Button.kR2.value, loop);
    }

    /**
     * Read the value of the R3 button on the controller.
     * 
     * @return The state of the button.
     */
    public boolean getR3Button() {
        return getRawButton(Button.kR3.value);
    }

    /**
     * Whether the R3 button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean getR3ButtonPressed() {
        return getRawButtonPressed(Button.kR3.value);
    }

    /**
     * Whether the R3 button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean getR3ButtonReleased() {
        return getRawButtonReleased(Button.kR3.value);
    }

    /**
     * Constructs an event instance around the R3 button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the R3 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent R3(EventLoop loop) {
        return button(Button.kR3.value, loop);
    }

    /**
     * Read the value of the L1 button on the controller.
     * 
     * @return The state of the button.
     */
    public boolean getL1Button() {
        return getRawButton(Button.kL1.value);
    }

    /**
     * Whether the L1 button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean getL1ButtonPressed() {
        return getRawButtonPressed(Button.kL1.value);
    }

    /**
     * Whether the L1 button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean getL1ButtonReleased() {
        return getRawButtonReleased(Button.kL1.value);
    }

    /**
     * Constructs an event instance around the L1 button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the L1 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent L1(EventLoop loop) {
        return button(Button.kL1.value, loop);
    }

    /**
     * Read the value of the L2 button on the controller.
     * 
     * @return The state of the button.
     */
    public boolean getL2Button() {
        return getRawButton(Button.kL2.value);
    }

    /**
     * Whether the L2 button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean getL2ButtonPressed() {
        return getRawButtonPressed(Button.kL2.value);
    }

    /**
     * Whether the L2 button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean getL2ButtonReleased() {
        return getRawButtonReleased(Button.kL2.value);
    }

    /**
     * Constructs an event instance around the L2 button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the L2 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent L2(EventLoop loop) {
        return button(Button.kL2.value, loop);
    }

    /**
     * Read the value of the L3 button on the controller.
     * 
     * @return The state of the button.
     */
    public boolean getL3Button() {
        return getRawButton(Button.kL3.value);
    }

    /**
     * Whether the L3 button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean getL3ButtonPressed() {
        return getRawButtonPressed(Button.kL3.value);
    }

    /**
     * Whether the L3 button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean getL3ButtonReleased() {
        return getRawButtonReleased(Button.kL3.value);
    }

    /**
     * Constructs an event instance around the L3 button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the L3 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent L3(EventLoop loop) {
        return button(Button.kL3.value, loop);
    }

    /**
     * Read the value of the 5 button on the controller.
     * 
     * @return The state of the button.
     */
    public boolean get5Button() {
        return getRawButton(Button.k5.value);
    }

    /**
     * Whether the 5 button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean get5ButtonPressed() {
        return getRawButtonPressed(Button.k5.value);
    }

    /**
     * Whether the 5 button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean get5ButtonReleased() {
        return getRawButtonReleased(Button.k5.value);
    }

    /**
     * Constructs an event instance around the 5 button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the 5 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent B5(EventLoop loop) {
        return button(Button.k5.value, loop);
    }

    /**
     * Read the value of the 6 button on the controller.
     * 
     * @return The state of the button.
     */
    public boolean get6Button() {
        return getRawButton(Button.k6.value);
    }

    /**
     * Whether the 6 button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean get6ButtonPressed() {
        return getRawButtonPressed(Button.k6.value);
    }

    /**
     * Whether the 6 button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean get6ButtonReleased() {
        return getRawButtonReleased(Button.k6.value);
    }

    /**
     * Constructs an event instance around the 6 button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the 6 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent B6(EventLoop loop) {
        return button(Button.k6.value, loop);
    }

    /**
     * Read the value of the 7 button on the controller.
     * 
     * @return The state of the button.
     */
    public boolean get7Button() {
        return getRawButton(Button.k7.value);
    }

    /**
     * Whether the 7 button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean get7ButtonPressed() {
        return getRawButtonPressed(Button.k7.value);
    }

    /**
     * Whether the 7 button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean get7ButtonReleased() {
        return getRawButtonReleased(Button.k7.value);
    }

    /**
     * Constructs an event instance around the 7 button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the 7 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent B7(EventLoop loop) {
        return button(Button.k7.value, loop);
    }

    /**
     * Read the value of the 8 button on the controller.
     * 
     * @return The state of the button.
     */
    public boolean get8Button() {
        return getRawButton(Button.k8.value);
    }

    /**
     * Whether the 8 button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean get8ButtonPressed() {
        return getRawButtonPressed(Button.k8.value);
    }

    /**
     * Whether the 8 button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean get8ButtonReleased() {
        return getRawButtonReleased(Button.k8.value);
    }

    /**
     * Constructs an event instance around the 8 button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the 8 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent B8(EventLoop loop) {
        return button(Button.k8.value, loop);
    }

    /**
     * Read the value of the start button on the controller.
     * 
     * @return The state of the button.
     */
    public boolean getStartButton() {
        return getRawButton(Button.kStart.value);
    }

    /**
     * Whether the start button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean getStartButtonPressed() {
        return getRawButtonPressed(Button.kStart.value);
    }

    /**
     * Whether the start button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean getStartButtonReleased() {
        return getRawButtonReleased(Button.kStart.value);
    }

    /**
     * Constructs an event instance around the start button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the start button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent start(EventLoop loop) {
        return button(Button.kStart.value, loop);
    }

    /**
     * Read the value of the select button on the controller.
     * 
     * @return The state of the button.
     */
    public boolean getSelectButton() {
        return getRawButton(Button.kSelect.value);
    }

    /**
     * Whether the select button was pressed since the last check.
     * 
     * @return Whether the button was pressed since the last check.
     */
    public boolean getSelectButtonPressed() {
        return getRawButtonPressed(Button.kSelect.value);
    }

    /**
     * Whether the select button was released since the last check.
     * 
     * @return Whether the button was released since the last check.
     */
    public boolean getSelectButtonReleased() {
        return getRawButtonReleased(Button.kSelect.value);
    }

    /**
     * Constructs an event instance around the select button's digital signal.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the select button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent select(EventLoop loop) {
        return button(Button.kSelect.value, loop);
    }
}
