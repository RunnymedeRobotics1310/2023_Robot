package frc.robot.commands.operator;


/**
 * The Runnymede 2023 Game Controller extends {@link RunnymedeGameController}
 * <p>
 * This class ... FIXME: Kaelin....
 * 
 */
public class Runnymede2023GameController extends RunnymedeGameController {

    /**
     * Construct a Runnymede2023GameController on the specified port
     * <p>
     * Uses the {{@link #DEFAULT_AXIS_DEADBAND} as the joystick deadband
     *
     * @param port on the driver station which the joystick is plugged into
     */
    public Runnymede2023GameController(int port) {
        super(port);
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
        super(port, axisDeadband);
    }

    public void sayHello() {
        System.out.println("2023 game controller says hi!");
    }

    public boolean isBoost() {
        return this.getLeftBumper();
    }

    public boolean isSlowDown() {
        return this.getRightBumper();
    }

    public boolean isHigh() {
        return this.getYButton();
    }

    public boolean isMid() {
        return this.getBButton();
    }

    public boolean isLow() {
        return this.getAButton();
    }

    public boolean isDrop() {
        return this.getXButton();
    }

    public boolean isPickUpCone() {
        return this.getLeftTriggerAxis() > 0.1;
    }

    public boolean isPickUpCube() {
        return this.getRightTriggerAxis() > 0.1;
    }

    public boolean isSubstation() {
        return this.getPOV() == 270;
    }

    public boolean isAdjustHigher() {
        return this.getPOV() == 0;
    }

    public boolean isAdjustLower() {
        return this.getPOV() == 180;
    }
}
