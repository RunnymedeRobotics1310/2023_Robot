package frc.robot.commands.operator;


/**
 * The DriverController exposes all driver functions
 */
public class OperatorInput {

    private final RunnymedeGameController controller;
    private final RunnymedeGameController auxController;

    public enum Stick {
        LEFT, RIGHT
    }

    ;

    public enum Axis {
        X, Y
    }

    ;

    /**
     * Construct an OperatorInput class that is fed by a DriverController and an AuxiliaryController.
     *
     * @param driverControllerPort on the driver station which the driver joystick is plugged into
     * @param auxControllerPort on the driver station which the aux joystick is plugged into
     */
    public OperatorInput(int driverControllerPort, int auxControllerPort) {
        controller    = new RunnymedeGameController(driverControllerPort);
        auxController = new RunnymedeGameController(auxControllerPort);
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
        return !shift() && controller.getLeftTriggerAxis() > 0.3;
    }

    public boolean isPickUpCube() {
        return !shift() && controller.getRightTriggerAxis() > 0.3;
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

    public double getAxis(Stick stick, Axis axis) {

        switch (stick) {

        case LEFT:
            switch (axis) {
            case X:
                return controller.getLeftX();
            case Y:
                return controller.getLeftY();
            }
            break;

        case RIGHT:
            switch (axis) {
            case X:
                return controller.getRightX();
            case Y:
                return controller.getRightY();
            }
            break;
        }

        return 0;
    }

    public boolean isCancel() {
        return controller.getStartButton();
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
            return controller.getLeftY() / 4;
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
     * return the raw underlying {@link RunnymedeGameController}. ONLY FOR USE IN TEST MODE.
     */
    public RunnymedeGameController getRawRunnymedeController() {
        return controller;
    }
}
