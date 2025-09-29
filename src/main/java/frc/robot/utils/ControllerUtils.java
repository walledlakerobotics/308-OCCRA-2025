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
     * Transforms a raw axis input based on a sensitivity, deadband, and
     * multiplier.
     * 
     * @param raw         The raw value from the axis in the range [-1, 1].
     * @param sensitivity The sensitivity factor. 1 does nothing, less than 1 makes
     *                    it less sensitive, greater than 1 makes it more sensitive.
     * @param max         The raw value will be multiplied by this after applying
     *                    deadband and sensitivity such that it will be the max
     *                    output value
     * @return The transformed axis value in the range [-max, max].
     */
    public static double axisTransform(double raw, double sensitivity, double max) {
        return max * Math.signum(raw) * Math.pow(raw, 1 / sensitivity);
    }
}
