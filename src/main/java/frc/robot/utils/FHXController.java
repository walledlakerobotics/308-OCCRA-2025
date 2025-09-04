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
        kR1(0),

        kL1(1),

        kR3(2),
        
        kL3(3),
        
        k5(4),
        
        k6(5),
        
        k7(6),
        
        k8(7),
        
        kR2(8),
        
        kL2(9),

        kSelect(10),

        kStart(11);


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
        /** Y axis. */
        kStickY(0),
        /** X axis. */
        kStickX(1),
        
        kThrottle(2),
        
        kStickRotation(5),

        kThrottleTilt(6),

        /*no no touch */
        kPOV(9);

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
    public double getStickXAxis() {
        return getRawAxis(Axis.kStickX.value);
    }

    public double getStickYAxis() {
        return getRawAxis(Axis.kStickY.value);
    }

    public double getThrottleAxis() {
        return getRawAxis(Axis.kThrottle.value);
    }

    public double getStickRotationAxis() {
        return getRawAxis(Axis.kStickRotation.value);
    }

    public double getThrottleTiltAxis() {
        return getRawAxis(Axis.kThrottleTilt.value);
    }

    public boolean getR1ButtonPressed() {
        return getRawButtonPressed(Button.kR1.value);
    }

    public boolean getR1ButtonReleased() {
        return getRawButtonPressed(Button.kR1.value);
    }

    //buttons
    public BooleanEvent r1(EventLoop event) {
        return button(Button.kR1.value, event);
    }

    public BooleanEvent r2(EventLoop event) {
        return button(Button.kR2.value, event);
    }

    public BooleanEvent r3(EventLoop event) {
        return button(Button.kR3.value, event);
    }

    public BooleanEvent l1(EventLoop event) {
        return button(Button.kL1.value, event);
    }

    public BooleanEvent l2(EventLoop event) {
        return button(Button.kL2.value, event);
    }

    public BooleanEvent l3(EventLoop event) {
        return button(Button.kL3.value, event);
    }

    public BooleanEvent select(EventLoop event) {
        return button(Button.kSelect.value, event);
    }

    public BooleanEvent start(EventLoop event) {
        return button(Button.kStart.value, event);
    }

    public BooleanEvent b5(EventLoop event) {
        return button(Button.k5.value, event);
    }

    public BooleanEvent b6(EventLoop event) {
        return button(Button.k6.value, event);
    }

    public BooleanEvent b7(EventLoop event) {
        return button(Button.k7.value, event);
    }

    public BooleanEvent b8(EventLoop event) {
        return button(Button.k8.value, event);
    }

    //triggers
    



}
