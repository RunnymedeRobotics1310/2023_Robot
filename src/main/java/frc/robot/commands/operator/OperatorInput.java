package frc.robot.commands.operator;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * The DriverController exposes all driver functions
 */
public class OperatorInput {

    private final RunnymedeGameController driverController;
    private final RunnymedeGameController operatorController;

    public enum Stick {
        LEFT, RIGHT
    }

    public enum Axis {
        X, Y
    }

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


    private boolean shift() {
        return driverController.getLeftBumper() && driverController.getRightBumper();
    }

    public boolean isBoost() {
        return driverController.getLeftBumper();
    }

    public boolean isSlowDown() {
        return driverController.getRightBumper();
    }

    public boolean isHigh() {
        return operatorController.getYButton();
    }

    public boolean isMid() {
        return operatorController.getBButton();
    }

    public boolean isLow() {
        return operatorController.getAButton();
    }

    public boolean isDrop() {
        return driverController.getXButton() || operatorController.getXButton();
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
        return driverController.getStartButton() || operatorController.getStartButton();
    }

    public boolean isCompact() {
        return operatorController.getBackButton();
    }

    public boolean isVisionReset() {
        return shift() && driverController.getBackButton() && !driverController.getStartButton();
    }

    public boolean isGyroReset() {
        return driverController.getBackButton() && !driverController.getStartButton();
    }

    public boolean isArmReset() {
        return shift() && driverController.getBackButton() && !driverController.getStartButton();
    }

    public boolean isToggleTestMode() {
        return !shift() && driverController.getBackButton() && driverController.getStartButton();
    }

    public boolean isVisionSubstationConePickup() {
        return driverController.getAButton();
    }

    public boolean isSubstationConePickup() {
        return driverController.getBButton();
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
        return operatorController.getLeftY();
    }

    public double getArmExtendMotorSpeed() {
        return operatorController.getLeftX();
    }

    public double getPincherMotorSpeed() {
        return operatorController.getRightY();
    }

    public boolean balance() {
        return operatorController.getPOV() == 0;
    }

    public boolean autoTune() {
        return operatorController.getPOV() == 90;
    }

    public boolean calibratePincher() {
        return operatorController.getPOV() == 270;
    }


    public boolean isCameraViewHigh() {
        return driverController.getPOV() == 0;
    }

    public boolean isCameraViewLow() {
        return driverController.getPOV() == 180 || operatorController.getPOV() == 180;
    }

    public void startVibrate() {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    }

    public void stopVibrate() {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }

    public boolean isPickUpCubeVision() {
        return driverController.getPOV() == 90;
    }

    /**
     * return the raw underlying {@link RunnymedeGameController}. ONLY FOR USE IN TEST MODE.
     */
    public RunnymedeGameController getRawDriverController() {
        return driverController;
    }
}
