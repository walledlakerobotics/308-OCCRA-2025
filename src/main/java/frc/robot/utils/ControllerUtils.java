// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * A utility class for controller inputs.
 */
public class ControllerUtils {
    private ControllerUtils() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Applies a sensitivity curve to a controller axis value.
     * 
     * @param raw         The raw value from the axis in the range [-1, 1].
     * @param sensitivity The sensitivity factor. 1 does nothing, less than 1 makes
     *                    it less sensitive, greater than 1 makes it more sensitive.
     * @return The adjusted axis value in the range [-1, 1].
     */
    public static double axisSensitivity(double raw, double sensitivity) {
        return Math.signum(raw) * Math.pow(Math.abs(raw), 1 / sensitivity);
    }
}
