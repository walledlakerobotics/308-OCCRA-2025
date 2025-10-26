package frc.robot.utils;

/**
 * A utility class for the Walled Lake Robotics CAN ID standard.
 */
public class CANIDs {
    private CANIDs() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * The roboRIO.
     * 
     * @return The CAN ID.
     */
    public static int roboRIO() {
        return 0;
    }

    /**
     * The power distribution hub (PDH).
     * 
     * @return The CAN ID.
     */
    public static int PDH() {
        return 1;
    }

    /**
     * The front left drive motor.
     * 
     * @return The CAN ID.
     */
    public static int frontLeft() {
        return 2;
    }

    /**
     * The front right drive motor.
     * 
     * @return The CAN ID.
     */
    public static int frontRight() {
        return 4;
    }

    /**
     * The rear right drive motor.
     * 
     * @return The CAN ID.
     */
    public static int rearRight() {
        return 6;
    }

    /**
     * The rear left drive motor.
     * 
     * @return The CAN ID.
     */
    public static int rearLeft() {
        return 8;
    }

    /**
     * The front left rotation motor.
     * 
     * @return The CAN ID.
     * @apiNote Only used in swerve drive.
     */
    public static int frontLeftRotation() {
        return frontLeft() + 1;
    }

    /**
     * The front right rotation motor.
     * 
     * @return The CAN ID.
     * @apiNote Only used in swerve drive.
     */
    public static int frontRightRotation() {
        return frontRight() + 1;
    }

    /**
     * The rear right rotation motor.
     * 
     * @return The CAN ID.
     * @apiNote Only used in swerve drive.
     */
    public static int rearRightRotation() {
        return rearRight() + 1;
    }

    /**
     * The rear left rotation motor.
     * 
     * @return The CAN ID.
     * @apiNote Only used in swerve drive.
     */
    public static int rearLeftRotation() {
        return rearLeft() + 1;
    }

    /**
     * The front left CANcoder.
     * 
     * @return The CAN ID.
     * @apiNote Only used in swerve drive.
     */
    public static int frontLeftCANcoder() {
        return 10 + frontLeftRotation();
    }

    /**
     * The front right CANcoder.
     * 
     * @return The CAN ID.
     * @apiNote Only used in swerve drive.
     */
    public static int frontRightCANcoder() {
        return 10 + frontRightRotation();
    }

    /**
     * The rear right CANcoder.
     * 
     * @return The CAN ID.
     * @apiNote Only used in swerve drive.
     */
    public static int rearRightCANcoder() {
        return 10 + rearRightRotation();
    }

    /**
     * The rear left CANcoder.
     * 
     * @return The CAN ID.
     * @apiNote Only used in swerve drive.
     */
    public static int rearLeftCANcoder() {
        return 10 + rearLeftRotation();
    }

    /**
     * A secondary (non-drive) motor.
     * 
     * @param deviceNum The device number from 0-9.
     * @return The CAN ID.
     */
    public static int secondaryMotor(int deviceNum) {
        return 20 + deviceNum;
    }

    /**
     * The pneumatics module.
     * 
     * @return The CAN ID.
     */
    public static int pneumaticsModule() {
        return pneumaticsModule(0);
    }

    /**
     * A pneumatics module.
     * 
     * @param moduleNum The module number form 0-9. Defaults to 0.
     * @return The CAN ID.
     */
    public static int pneumaticsModule(int moduleNum) {
        return 30 + moduleNum;
    }

    /**
     * A miscellaneous CAN device.
     * 
     * @param deviceNum The device number from 0-9.
     * @return The CAN ID.
     */
    public static int miscellaneousDevice(int deviceNum) {
        return 40 + deviceNum;
    }
}
