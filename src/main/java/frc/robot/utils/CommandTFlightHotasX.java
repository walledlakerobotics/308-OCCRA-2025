// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A version of {@link TFlightHotasX} with {@link Trigger} factories for
 * command-based.
 *
 * @see TFlightHotasX
 */
public class CommandTFlightHotasX extends CommandGenericHID {
    private final TFlightHotasX m_hid;

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is
     *             plugged into.
     */
    public CommandTFlightHotasX(int port) {
        super(port);
        m_hid = new TFlightHotasX(port);
    }

    /**
     * Get the underlying GenericHID object.
     *
     * @return the wrapped GenericHID object
     */
    @Override
    public TFlightHotasX getHID() {
        return m_hid;
    }

    /**
     * Constructs a Trigger instance around the R1 button's digital signal.
     *
     * @return a Trigger instance representing the R1 button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #R1(EventLoop)
     */
    public Trigger R1() {
        return R1(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the R1 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the R1 button's digital signal
     *         attached to the given loop.
     */
    public Trigger R1(EventLoop loop) {
        return button(TFlightHotasX.Button.kR1.value, loop);
    }

    /**
     * Constructs a Trigger instance around the R2 button's digital signal.
     *
     * @return a Trigger instance representing the R2 button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #R2(EventLoop)
     */
    public Trigger R2() {
        return R2(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the R2 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the R2 button's digital signal
     *         attached to the given loop.
     */
    public Trigger R2(EventLoop loop) {
        return button(TFlightHotasX.Button.kR2.value, loop);
    }

    /**
     * Constructs a Trigger instance around the R3 button's digital signal.
     *
     * @return a Trigger instance representing the R3 button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #R3(EventLoop)
     */
    public Trigger R3() {
        return R3(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the R3 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the R3 button's digital signal
     *         attached to the given loop.
     */
    public Trigger R3(EventLoop loop) {
        return button(TFlightHotasX.Button.kR3.value, loop);
    }

    /**
     * Constructs a Trigger instance around the L1 button's digital signal.
     *
     * @return a Trigger instance representing the L1 button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #L1(EventLoop)
     */
    public Trigger L1() {
        return L1(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the L1 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the L1 button's digital signal
     *         attached to the given loop.
     */
    public Trigger L1(EventLoop loop) {
        return button(TFlightHotasX.Button.kL1.value, loop);
    }

    /**
     * Constructs a Trigger instance around the L2 button's digital signal.
     *
     * @return a Trigger instance representing the L2 button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #L2(EventLoop)
     */
    public Trigger L2() {
        return L2(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the L2 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the L2 button's digital signal
     *         attached to the given loop.
     */
    public Trigger L2(EventLoop loop) {
        return button(TFlightHotasX.Button.kL2.value, loop);
    }

    /**
     * Constructs a Trigger instance around the L3 button's digital signal.
     *
     * @return a Trigger instance representing the L3 button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #L3(EventLoop)
     */
    public Trigger L3() {
        return L3(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the L3 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the L3 button's digital signal
     *         attached to the given loop.
     */
    public Trigger L3(EventLoop loop) {
        return button(TFlightHotasX.Button.kL3.value, loop);
    }

    /**
     * Constructs a Trigger instance around the 5 button's digital signal.
     *
     * @return a Trigger instance representing the 5 button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #B5(EventLoop)
     */
    public Trigger B5() {
        return B5(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the 5 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the 5 button's digital signal
     *         attached to the given loop.
     */
    public Trigger B5(EventLoop loop) {
        return button(TFlightHotasX.Button.k5.value, loop);
    }

    /**
     * Constructs a Trigger instance around the 6 button's digital signal.
     *
     * @return a Trigger instance representing the 6 button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #B6(EventLoop)
     */
    public Trigger B6() {
        return B6(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the 66 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the R1 button's digital signal
     *         attached to the given loop.
     */
    public Trigger B6(EventLoop loop) {
        return button(TFlightHotasX.Button.k6.value, loop);
    }

    /**
     * Constructs a Trigger instance around the 7 button's digital signal.
     *
     * @return a Trigger instance representing the 7 button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #B7(EventLoop)
     */
    public Trigger B7() {
        return B7(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the 7 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the 7 button's digital signal
     *         attached to the given loop.
     */
    public Trigger B7(EventLoop loop) {
        return button(TFlightHotasX.Button.k7.value, loop);
    }

    /**
     * Constructs a Trigger instance around the 8 button's digital signal.
     *
     * @return a Trigger instance representing the 8 button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #B8(EventLoop)
     */
    public Trigger B8() {
        return B8(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the 8 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the 8 button's digital signal
     *         attached to the given loop.
     */
    public Trigger B8(EventLoop loop) {
        return button(TFlightHotasX.Button.k8.value, loop);
    }

    /**
     * Constructs a Trigger instance around the select button's digital signal.
     *
     * @return a Trigger instance representing the select button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #select(EventLoop)
     */
    public Trigger select() {
        return select(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the select button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the select button's digital signal
     *         attached to the given loop.
     */
    public Trigger select(EventLoop loop) {
        return button(TFlightHotasX.Button.kSelect.value, loop);
    }

    /**
     * Constructs a Trigger instance around the start button's digital signal.
     *
     * @return a Trigger instance representing the start button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #start(EventLoop)
     */
    public Trigger start() {
        return start(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the start button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the start button's digital signal
     *         attached to the given loop.
     */
    public Trigger start(EventLoop loop) {
        return button(TFlightHotasX.Button.kStart.value, loop);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left rocker. The
     * returned trigger will be true when the axis value is less than
     * {@code -threshold}.
     * 
     * @param loop      the event loop instance to attach the event to.
     * @param threshold the negative of the maximum axis value for the returned
     *                  {@link Trigger} to be true. This value should be in the
     *                  range [0, 1] where 0 is the unpressed state of the axis.
     * @return a Trigger instace that is true when the rockers's axis is below
     *         the negative of the provided threshold, attached to the given event
     *         loop.
     */
    public Trigger getLeftRockerTrigger(double threshold, EventLoop loop) {
        return axisLessThan(TFlightHotasX.Axis.kRocker.value, -threshold, loop);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left rocker. The
     * returned trigger will be true when the axis value is less than
     * {@code -threshold}.
     * 
     * @param threshold the negative of the maximum axis value for the returned
     *                  {@link Trigger} to be true. This value should be in the
     *                  range [0, 1] where 0 is the unpressed state of the axis.
     * @return a Trigger instace that is true when the rockers's axis is below
     *         the negative of the provided threshold, attached to the
     *         {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     */
    public Trigger getLeftRockerTrigger(double threshold) {
        return getLeftRockerTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the axis value of the rocker. The
     * returned trigger will be true when the axis value is less than -0.5.
     * 
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance that is true when the rockers's axis is below
     *         the negative of the provided threshold, attached to the
     *         {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     */
    public Trigger getLeftRockerTrigger() {
        return getLeftRockerTrigger(0.5, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger around the axis value of the rocker. The
     * returned trigger will be true when the axis value is greater than
     * {@code threshold}.
     * 
     * @param loop      the event loop instance to attach the event to.
     * @param threshold the minimum axis value for the returned {@link Trigger}
     *                  to be true. This value should be in the range [0, 1] where 0
     *                  is the unpressed state of the axis.
     * @return a Trigger instance that is true when the rockers's axis exceeds
     *         the provided threshold, attached to the given event loop.
     */
    public Trigger getRightRockerTrigger(double threshold, EventLoop loop) {
        return axisGreaterThan(TFlightHotasX.Axis.kRocker.value, threshold, loop);
    }

    /**
     * Constructs a Trigger around the axis value of the rocker. The
     * returned trigger will be true when the axis value is greater than
     * {@code threshold}.
     * 
     * @param threshold the minimum axis value for the returned {@link Trigger}
     *                  to be true. This value should be in the range [0, 1] where 0
     *                  is the unpressed state of the axis.
     * @return a Trigger instance that is true when the rockers's axis exceeds
     *         the provided threshold, attached to the
     *         {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     */
    public Trigger getRightRockerTrigger(double threshold) {
        return getLeftRockerTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the axis value of the rocker. The
     * returned trigger will be true when the axis value is greater than
     * 0.5.
     * 
     * @return an event instance that is true when the rockers's axis exceeds
     *         the provided threshold, attached to the
     *         {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     */
    public Trigger getRightRockerTrigger() {
        return getLeftRockerTrigger(0.5, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Get the X axis value of the joystick of the controller. Right is positive.
     * 
     * @return The axis value.
     */
    public double getStickX() {
        return m_hid.getStickX();
    }

    /**
     * Get the Y axis value of the joystick of the controller. Back is positive.
     * 
     * @return The axis value.
     */
    public double getStickY() {
        return m_hid.getStickY();
    }

    /**
     * Get the throttle axis value of the controller. Forward is positive.
     * 
     * @return The axis value.
     */
    public double getThrottle() {
        return m_hid.getThrottle();
    }

    /**
     * Get the rudder axis value of the controller. Right is positive.
     * 
     * @return The axis value.
     */
    public double getRudder() {
        return m_hid.getRudder();
    }

    /**
     * Get the rocker axis value of the controller. Right is positive.
     * 
     * @return The axis value.
     */
    public double getRockerAxis() {
        return m_hid.getRockerAxis();
    }
}
