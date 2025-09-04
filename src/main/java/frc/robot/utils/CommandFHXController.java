package frc.robot.utils;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandFHXController extends CommandGenericHID {

    private final FHXController m_hid;

    public CommandFHXController(int port) {
        super(port);
        m_hid = new FHXController(port);
    }
    
    /**
   * Get the underlying GenericHID object.
   *
   * @return the wrapped GenericHID object
   */
    @Override
    public FHXController getHID() {
        return m_hid;
    }

    public Trigger r1() {
        return r1(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger r1(EventLoop loop) {
        return button(FHXController.Button.kR1.value, loop);
    }

    public Trigger r2() {
        return r2(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger r2(EventLoop loop) {
        return button(FHXController.Button.kR2.value, loop);
    }

    public Trigger r3() {
        return r3(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger r3(EventLoop loop) {
        return button(FHXController.Button.kR3.value, loop);
    }

    public Trigger l1() {
        return l1(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger l1(EventLoop loop) {
        return button(FHXController.Button.kL1.value, loop);
    }

    public Trigger l2() {
        return l2(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger l2(EventLoop loop) {
        return button(FHXController.Button.kL2.value, loop);
    }

    public Trigger l3() {
        return l3(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger l3(EventLoop loop) {
        return button(FHXController.Button.kL3.value, loop);
    }

    public Trigger b5() {
        return b5(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger b5(EventLoop loop) {
        return button(FHXController.Button.k5.value, loop);
    }

    public Trigger b6() {
        return b6(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger b6(EventLoop loop) {
        return button(FHXController.Button.k6.value, loop);
    }

    public Trigger b7() {
        return b7(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger b7(EventLoop loop) {
        return button(FHXController.Button.k7.value, loop);
    }

    public Trigger b8() {
        return b8(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger b8(EventLoop loop) {
        return button(FHXController.Button.k8.value, loop);
    }

    public Trigger select() {
        return select(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger select(EventLoop loop) {
        return button(FHXController.Button.kSelect.value, loop);
    }

    public Trigger start() {
        return start(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger start(EventLoop loop) {
        return button(FHXController.Button.kStart.value, loop);
    }

    public Trigger getLeftRockerTrigger(double threshold, EventLoop loop) {
        return axisLessThan(FHXController.Axis.kRocker.value, -threshold, loop);
    }

    public Trigger getLeftRockerTrigger(double threshold) {
        return getLeftRockerTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger getLeftRockerTrigger() {
        return getLeftRockerTrigger(0.5, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger getRightRockerTrigger(double threshold, EventLoop loop) {
        return axisGreaterThan(FHXController.Axis.kRocker.value, threshold, loop);
    }

    public Trigger getRightRockerTrigger(double threshold) {
        return getLeftRockerTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger getRightRockerTrigger() {
        return getLeftRockerTrigger(0.5, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public double getStickX() {
        return m_hid.getStickX();
    }

    public double getStickY() {
        return m_hid.getStickY();
    }

    public double getThrottle() {
        return m_hid.getThrottle();
    }

    public double getRudder() {
        return m_hid.getRudder();
    }

    public double getRockerAxis() {
        return m_hid.getRockerAxis();
    }
}
