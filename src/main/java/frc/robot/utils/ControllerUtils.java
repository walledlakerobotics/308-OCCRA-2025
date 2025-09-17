package frc.robot.utils;

public class ControllerUtils {
    private ControllerUtils() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static double joystickTransform(double raw, double sensitivity, double deadband, double multiplier) {
        raw *= multiplier;
        return Math.signum(raw) * Math.pow(Math.max(0, (Math.abs(raw) - deadband) / (1 - deadband)), 1 / sensitivity);
    }
}
