package frc.robot.utils;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class FHXController extends GenericHID {
/** Represents a digital button on a XboxController. */

    public FHXController(int port) {
        super(port);
        HAL.report(tResourceType.kResourceType_Controller, port + 1);
    }

    public enum Button {
        kR1(1),
        kR2(9),
        kR3(3),
        kL1(2),
        kL2(10),
        kL3(4),
        k5(5),
        k6(6),
        k7(7),
        k8(8),
        kStart(12),
        kSelect(11);


        /** Button value. */
        public final int value;

        Button(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the button, matching the relevant methods. This is done by
         * stripping the leading `k`, and appending `Button`.
         *
         * <p>Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the button.
         */
        @Override
        public String toString() {
        // Remove leading `k`
            return this.name().substring(1) + "Button";
        }
    }

  /** Represents an axis on an XboxController. */
    public enum Axis {
        /** X axis. */
        kStickX(1),

        /** Y axis. */
        kStickY(0),
        
        kThrottle(2),
        
        kRudder(3),

        kRocker(4);

        /** Axis value. */
        public final int value;

        Axis(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the axis, matching the relevant methods. This is done by
         * stripping the leading `k`, and appending `Axis` if the name ends with `Trigger`.
         *
         * <p>Primarily used for automated unit tests.
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


    // getters  
    public double getStickX() {
        return -getRawAxis(Axis.kStickX.value);
    }

    public double getStickY() {
        return getRawAxis(Axis.kStickY.value);
    }

    public double getThrottle() {
        return -getRawAxis(Axis.kThrottle.value);
    }

    public double getRudder() {
        return getRawAxis(Axis.kRudder.value);
    }

    public double getRockerAxis() {
        return getRawAxis(Axis.kRocker.value);
    }

    public BooleanEvent getLeftRockerTrigger(double threshold, EventLoop loop) {
        return axisLessThan(Axis.kRocker.value, -threshold, loop);
    }

    public BooleanEvent getLeftRockerTrigger(EventLoop loop) {
        return getLeftRockerTrigger(0.5, loop);
    }

    public BooleanEvent getRightRockerTrigger(double threshold, EventLoop loop) {
        return axisGreaterThan(Axis.kRocker.value, threshold, loop);
    }

    public BooleanEvent getRightRockerTrigger(EventLoop loop) {
        return getLeftRockerTrigger(0.5, loop);
    }

    public boolean getR1Button() {
        return getRawButton(Button.kR1.value);
    }

    public boolean getR1ButtonPressed() {
        return getRawButtonPressed(Button.kR1.value);
    }

    public boolean getR1ButtonReleased() {
        return getRawButtonReleased(Button.kR1.value);
    }

    public BooleanEvent R1(EventLoop event) {
        return button(Button.kR1.value, event);
    }

    public boolean getR2Button() {
        return getRawButton(Button.kR2.value);
    }

    public boolean getR2ButtonPressed() {
        return getRawButtonPressed(Button.kR2.value);
    }

    public boolean getR2ButtonReleased() {
        return getRawButtonReleased(Button.kR2.value);
    }

    public BooleanEvent R2(EventLoop event) {
        return button(Button.kR2.value, event);
    }

    public boolean getR3Button() {
        return getRawButton(Button.kR3.value);
    }

    public boolean getR3ButtonPressed() {
        return getRawButtonPressed(Button.kR3.value);
    }

    public boolean getR3ButtonReleased() {
        return getRawButtonReleased(Button.kR3.value);
    }

    public BooleanEvent R3(EventLoop event) {
        return button(Button.kR3.value, event);
    }

    public boolean getL1Button() {
        return getRawButton(Button.kL1.value);
    }

    public boolean getL1ButtonPressed() {
        return getRawButtonPressed(Button.kL1.value);
    }

    public boolean getL1ButtonReleased() {
        return getRawButtonReleased(Button.kL1.value);
    }

    public BooleanEvent L1(EventLoop event) {
        return button(Button.kL1.value, event);
    }

    public boolean getL2Button() {
        return getRawButton(Button.kL2.value);
    }

    public boolean getL2ButtonPressed() {
        return getRawButtonPressed(Button.kL2.value);
    }

    public boolean getL2ButtonReleased() {
        return getRawButtonReleased(Button.kL2.value);
    }

    public BooleanEvent L2(EventLoop event) {
        return button(Button.kL2.value, event);
    }

    public boolean getL3Button() {
        return getRawButton(Button.kL3.value);
    }
    
    public boolean getL3ButtonPressed() {
        return getRawButtonPressed(Button.kL3.value);
    }

    public boolean getL3ButtonReleased() {
        return getRawButtonReleased(Button.kL3.value);
    }

    public BooleanEvent L3(EventLoop event) {
        return button(Button.kL3.value, event);
    }

    public boolean getButton5() {
        return getRawButton(Button.k5.value);
    }

    public boolean getButton5Pressed() {
        return getRawButtonPressed(Button.k5.value);
    }

    public boolean getButton5Released() {
        return getRawButtonReleased(Button.k5.value);
    }

    public BooleanEvent B5(EventLoop event) {
        return button(Button.k5.value, event);
    }

    public boolean getButton6() {
        return getRawButton(Button.k6.value);
    }

    public boolean getButton6Pressed() {
        return getRawButtonPressed(Button.k6.value);
    }

    public boolean getButton6Released() {
        return getRawButtonReleased(Button.k6.value);
    }

    public BooleanEvent B6(EventLoop event) {
        return button(Button.k6.value, event);
    }

    public boolean getButton7() {
        return getRawButton(Button.k7.value);
    }
    
    public boolean getButton7Pressed() {
        return getRawButtonPressed(Button.k7.value);
    }

    public boolean getButton7Released() {
        return getRawButtonReleased(Button.k7.value);
    }

    public BooleanEvent B7(EventLoop event) {
        return button(Button.k7.value, event);
    }

    public boolean getButton8() {
        return getRawButton(Button.k8.value);
    }
    
    public boolean getButton8Pressed() {
        return getRawButtonPressed(Button.k8.value);
    }

    public boolean getButton8Released() {
        return getRawButtonReleased(Button.k8.value);
    }

    public BooleanEvent B8(EventLoop event) {
        return button(Button.k8.value, event);
    }

    public boolean getStartButton() {
        return getRawButton(Button.kStart.value);
    }

    public boolean getStartButtonPressed() {
        return getRawButtonPressed(Button.kStart.value);
    }

    public boolean getStartButtonReleased() {
        return getRawButtonReleased(Button.kStart.value);
    }

    public BooleanEvent start(EventLoop event) {
        return button(Button.kStart.value, event);
    }

    public boolean getSelectButton() {
        return getRawButton(Button.kSelect.value);
    }

    public boolean getSelectButtonPressed() {
        return getRawButtonPressed(Button.kSelect.value);
    }

    public boolean getSelectButtonReleased() {
        return getRawButtonReleased(Button.kSelect.value);
    }

    public BooleanEvent select(EventLoop event) {
        return button(Button.kSelect.value, event);
    }
}
