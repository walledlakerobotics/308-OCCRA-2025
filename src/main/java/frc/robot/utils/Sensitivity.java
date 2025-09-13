package frc.robot.utils;

public class Sensitivity {
    private Sensitivity() {}

    public static double sensitivity(double raw, double sensitivity, double deadband) {
        return Math.signum(raw) * Math.pow(Math.max(0,(Math.abs(raw) - deadband) / (1 - deadband)), 0.5 / sensitivity);
    }
}
