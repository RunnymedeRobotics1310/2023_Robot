package frc.robot.commands.operator;


/**
 * The DriverController exposes all driver functions
 */
public class OperatorInput {

    private final RunnymedeGameController driverController;
    private final RunnymedeGameController operatorController;

    public enum Stick {
        LEFT, RIGHT
    }

    ;

    public enum Axis {
        X, Y
    }

    ;

    /**
     * Construct an OperatorInput class that is fed by a DriverController and an OperatorController.
     *
     * @param driverControllerPort on the driver station which the driver joystick is plugged into
     * @param operatorControllerPort on the driver station which the aux joystick is plugged into
     */
    public OperatorInput(int driverControllerPort, int operatorControllerPort) {
        driverController   = new RunnymedeGameController(driverControllerPort);
        operatorController = new RunnymedeGameController(operatorControllerPort);
    }


    public void sayHello() {
        System.out.println("2023 game controller says hi!");
    }

    private boolean dcShift() {
        return driverController.getLeftBumper() && driverController.getRightBumper();
    }

    public boolean isBoost() {
        return driverController.getLeftBumper();
    }

    public boolean isSlowDown() {
        return driverController.getRightBumper();
    }

    public boolean isHigh() {
        return driverController.getYButton();
    }

    public boolean isMid() {
        return driverController.getBButton();
    }

    public boolean isLow() {
        return driverController.getAButton();
    }

    public boolean isDrop() {
        return driverController.getXButton();
    }

    public boolean isPickUpCone() {
        return driverController.getLeftTriggerAxis() > 0.3;
    }

    public boolean isPickUpCube() {
        return driverController.getRightTriggerAxis() > 0.3;
    }

    public double getDriverControllerAxis(Stick stick, Axis axis) {

        switch (stick) {

        case LEFT:
            switch (axis) {
            case X:
                return driverController.getLeftX();
            case Y:
                return driverController.getLeftY();
            }
            break;

        case RIGHT:
            switch (axis) {
            case X:
                return driverController.getRightX();
            case Y:
                return driverController.getRightY();
            }
            break;
        }

        return 0;
    }

    public boolean isCancel() {
        return driverController.getStartButton();
    }

    public boolean isVisionReset() {
        return dcShift() && driverController.getBackButton() && !driverController.getStartButton();
    }

    public boolean isGyroReset() {
        return driverController.getBackButton() && !driverController.getStartButton();
    }

    public boolean isArmReset() {
        return dcShift() && driverController.getBackButton() && !driverController.getStartButton();
    }

    public boolean isToggleTestMode() {
        return !dcShift() && driverController.getBackButton() && driverController.getStartButton();
    }

    public double getCameraMotorSpeed() {
        if (operatorController.getLeftTriggerAxis() > 0 && operatorController.getRightTriggerAxis() > 0) {
            return 0;
        }
        if (operatorController.getLeftTriggerAxis() > 0) {
            return -operatorController.getLeftTriggerAxis();
        }
        if (operatorController.getRightTriggerAxis() > 0) {
            return operatorController.getRightTriggerAxis();
        }
        return 0;
    }

    public double getArmLiftMotorSpeed() {
        return operatorController.getLeftY() / 4;
    }

    public double getArmExtendMotorSpeed() {
        return operatorController.getLeftX();
    }

    public double getPincerMotorSpeed() {
        return operatorController.getRightY();
    }

    /**
     * return the raw underlying {@link RunnymedeGameController}. ONLY FOR USE IN TEST MODE.
     */
    public RunnymedeGameController getRawDriverController() {
        return driverController;
    }
}
