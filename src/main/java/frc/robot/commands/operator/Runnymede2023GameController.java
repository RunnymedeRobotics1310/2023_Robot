package frc.robot.commands.operator;


/**
 * The Runnymede 2023 Game Controller extends {@link RunnymedeGameController}
 * <p>
 * This class ... FIXME: Kaelin....
 * 
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
     * @param axisDeadband (0 - 0.4) to use for all axis values on this controller. When the
     * axis value from the hardware is less than the specified value, then the axis will return
     * zero. Setting the axisDeadbanding to zero turns off all deadbanding.
     * Values < 0 or > 0.4 are ignored, and
     * the {@link #DEFAULT_AXIS_DEADBAND} value is used.
     */
    public Runnymede2023GameController(int port, final double axisDeadband) {
        controller = new RunnymedeGameController(port, axisDeadband);
    }

    public void sayHello() {
        System.out.println("2023 game controller says hi!");
    }

    public boolean isBoost() {
        return controller.getLeftBumper();
    }

    public boolean isSlowDown() {
        return controller.getRightBumper();
    }

    public boolean isHigh() {
        return controller.getYButton();
    }

    public boolean isMid() {
        return controller.getBButton();
    }

    public boolean isLow() {
        return controller.getAButton();
    }

    public boolean isDrop() {
        return controller.getXButton();
    }

    public boolean isPickUpCone() {
        return controller.getLeftTriggerAxis() > 0.1;
    }

    public boolean isPickUpCube() {
        return controller.getRightTriggerAxis() > 0.1;
    }

    public boolean isSubstation() {
        return controller.getPOV() == 270;
    }

    public boolean isAdjustHigher() {
        return controller.getPOV() == 0;
    }

    public boolean isAdjustLower() {
        return controller.getPOV() == 180;
    }

    public boolean isUnnamed() {
        return controller.getPOV() == 90;
    }

    public double leftX() {
        return controller.getLeftX();
    }

    public double leftY() {
        return controller.getLeftY();
    }

    public double rightX() {
        return controller.getRightX();
    }

    public double rightY() {
        return controller.getRightY();
    }

    public boolean isCancel() {
        return controller.getStartButton();
    }

    public boolean isGyroReset() {
        return controller.getBackButton();
    }

    public boolean isToggleTestMode() {
        return controller.getBackButton() && controller.getStartButton();
    }
}
