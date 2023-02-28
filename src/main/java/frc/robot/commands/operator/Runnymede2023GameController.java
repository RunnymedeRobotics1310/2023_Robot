package frc.robot.commands.operator;


/**
 * The Runnymede 2023 Game Controller extends {@link RunnymedeGameController}
 * <p>
 * This class ... FIXME: Kaelin....
 */
public class Runnymede2023GameController {

    private final RunnymedeGameController controller;

    /**
     * Construct a Runnymede2023GameController on the specified port
     * <p>
     * Uses the {{@link #DEFAULT_AXIS_DEADBAND} as the joystick deadband
     *
     * @param port on the driver station which the joystick is plugged into
     */
    public Runnymede2023GameController(int port) {
        controller = new RunnymedeGameController(port);
    }

    /**
     * Construct a Runnymede2023GameController on the specified port with the specified deadband
     *
     * @param port on the driver station which the joystick is plugged into
     * @param axisDeadband (0 - 0.4) to use for all axis values on this controller. When the axis value from the hardware is less
     * than the specified value, then the axis will return zero. Setting the axisDeadbanding to zero turns off all deadbanding.
     * Values < 0 or > 0.4 are ignored, and the {@link #DEFAULT_AXIS_DEADBAND} value is used.
     */
    public Runnymede2023GameController(int port, final double axisDeadband) {
        controller = new RunnymedeGameController(port, axisDeadband);
    }

    public void sayHello() {
        System.out.println("2023 game controller says hi!");
    }

    private boolean shift() {
        return controller.getLeftBumper() && controller.getRightBumper();
    }

    public boolean isBoost() {
        return !shift() && controller.getLeftBumper();
    }

    public boolean isSlowDown() {
        return !shift() && controller.getRightBumper();
    }

    public boolean isHigh() {
        return !shift() && controller.getYButton();
    }

    public boolean isMid() {
        return !shift() && controller.getBButton();
    }

    public boolean isLow() {
        return !shift() && controller.getAButton();
    }

    public boolean isDrop() {
        return !shift() && controller.getXButton();
    }

    public boolean isPickUpCone() {
        return !shift() && controller.getLeftTriggerAxis() > 0.1;
    }

    public boolean isPickUpCube() {
        return !shift() && controller.getRightTriggerAxis() > 0.1;
    }

    public boolean isSubstation() {
        return !shift() && controller.getPOV() == 270;
    }

    public boolean isAdjustHigher() {
        return !shift() && controller.getPOV() == 0;
    }

    public boolean isAdjustLower() {
        return !shift() && controller.getPOV() == 180;
    }

    public boolean isUnnamed() {
        return !shift() && controller.getPOV() == 90;
    }

    public double getRawAxis(int axisID) {
        return controller.getRawAxis(axisID);
    }

    public boolean isCancel() {
        return controller.getStartButton() && !controller.getBackButton();
    }

    public boolean isVisionReset() {
        return shift() && controller.getBackButton() && !controller.getStartButton();
    }

    public boolean isGyroReset() {
        return controller.getBackButton() && !controller.getStartButton();
    }

    public boolean isArmReset() {
        return shift() && controller.getBackButton() && !controller.getStartButton();
    }

    public boolean isToggleTestMode() {
        return !shift() && controller.getBackButton() && controller.getStartButton();
    }

    public double getCameraMotorSpeed() {
        if (shift()) {
            if (controller.getLeftTriggerAxis() > 0 && controller.getRightTriggerAxis() > 0) {
                return 0;
            }
            if (controller.getLeftTriggerAxis() > 0) {
                return -controller.getLeftTriggerAxis();
            }
            if (controller.getRightTriggerAxis() > 0) {
                return controller.getRightTriggerAxis();
            }
        }
        return 0;
    }

    public double getArmLiftMotorSpeed() {
        if (shift()) {
            return controller.getLeftY();
        }
        return 0;
    }

    public double getArmExtendMotorSpeed() {
        if (shift()) {
            return controller.getLeftX();
        }
        return 0;
    }

    public double getPincerMotorSpeed() {
        if (shift()) {
            return controller.getRightY();
        }
        return 0;
    }

    /**
     * return raw runnymede controller. ONLY FOR USE IN TEST MODE.
     */
    public RunnymedeGameController getRawRunnymedeController() {
        return controller;
    }
}
