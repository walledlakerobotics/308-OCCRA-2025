package frc.robot.utils;

public class ControllerUtils {
    private ControllerUtils() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static double sensitivity(double raw, double sensitivity, double deadband) {
        return Math.signum(raw) * Math.pow(Math.max(0, (Math.abs(raw) - deadband) / (1 - deadband)), 1 / sensitivity);
    }
}
