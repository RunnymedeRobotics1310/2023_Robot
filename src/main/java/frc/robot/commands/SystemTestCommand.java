package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.commands.operator.RunnymedeGameController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This command is used to safely stop the robot in its current position, and to cancel any running
 * commands
 */
public class SystemTestCommand extends CommandBase {

    private enum Motor {
        NONE,
        DRIVE_LEFT_1, DRIVE_LEFT_2, DRIVE_LEFT,
        DRIVE_RIGHT_1, DRIVE_RIGHT_2, DRIVE_RIGHT,
        ARM_LIFT_1, ARM_LIFT_2, ARM_LIFT, ARM_LIFT_PID,
        ARM_EXTEND,
        PINCHER,
        CAMERA
    }

    private final OperatorInput           driverController;
    private final RunnymedeGameController controller;
    private final DriveSubsystem          driveSubsystem;
    private final ArmSubsystem            armSubsystem;
    private final VisionSubsystem         visionSubsystem;

    private long                          startTime           = 0;
    private Motor                         selectedMotor       = Motor.NONE;
    private double                        povMotorSpeed       = 0;

    private boolean                       previousLeftBumper  = false;
    private boolean                       previousRightBumper = false;

    /**
     * System Test Command
     *
     * All subsystems must be passed to this command, and each subsystem should have a stop command
     * that safely stops the robot
     * from moving.
     */
    public SystemTestCommand(OperatorInput driverController, DriveSubsystem driveSubsystem,
        ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem) {

        this.driverController = driverController;
        this.controller       = driverController.getRawDriverController();
        this.driveSubsystem   = driveSubsystem;
        this.armSubsystem     = armSubsystem;
        this.visionSubsystem  = visionSubsystem;

        addRequirements(driveSubsystem, armSubsystem, visionSubsystem);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        /*
         * The SystemTestCommand is not interruptible, and prevents all other commands that try to
         * interrupt it. Only the cancel button will end the SystemTestCommand.
         */
        return InterruptionBehavior.kCancelIncoming;
    }

    @Override
    public void initialize() {

        System.out.println("SystemTestCommand : started.");

        stopAllMotors();

        startTime = System.currentTimeMillis();

        SmartDashboard.putBoolean("Test Mode", true);
        SmartDashboard.putString("Test Motor", selectedMotor.toString());
        SmartDashboard.putNumber("Test Motor Speed", povMotorSpeed);

        clearMotorIndicators();
    }

    @Override
    public void execute() {

        /*
         * Use the bumpers to select the next / previous motor in the motor ring.
         *
         * Switching motors will cause all motors to stop
         */

        boolean rightBumper = controller.getRightBumper() && !previousRightBumper;
        previousRightBumper = controller.getRightBumper();

        boolean leftBumper = controller.getLeftBumper() && !previousLeftBumper;
        previousLeftBumper = controller.getLeftBumper();

        if (rightBumper || leftBumper) {

            int nextMotorIndex = selectedMotor.ordinal();

            if (rightBumper) {

                // Select the next motor in the ring
                nextMotorIndex = (nextMotorIndex + 1) % Motor.values().length;
            }
            else {

                // Select the previous motor in the ring
                nextMotorIndex--;
                if (nextMotorIndex < 0) {
                    nextMotorIndex = Motor.values().length - 1;
                }
            }

            clearMotorIndicators();
            stopAllMotors();

            selectedMotor = Motor.values()[nextMotorIndex];

            SmartDashboard.putString("Test Motor", selectedMotor.toString());
        }

        /*
         * The SystemTestCommand can use either the POV or the triggers to control
         * the motor speed. If the triggers are used, the POV is cleared.
         *
         * Once the motor is selected, use the POV up and down to
         * adjust the motor speed.
         *
         * The speed is adjusted 50 times / second as the user holds the
         * POV control. Allow a 5 seconds to ramp the speed from 0 to full value.
         *
         * increment = 1.0 (full) / 50 adjustments/sec / 5 sec = .004 adjustment size / loop.
         */

        int    pov          = controller.getPOV();
        double leftTrigger  = controller.getLeftTriggerAxis();
        double rightTrigger = controller.getRightTriggerAxis();

        double motorSpeed   = 0;

        if (leftTrigger > 0 && rightTrigger > 0) {

            // If both triggers are pressed, then stop the motor
            motorSpeed    = 0;
            povMotorSpeed = 0;
        }
        else if (leftTrigger > 0) {

            motorSpeed    = -leftTrigger;
            povMotorSpeed = 0;
        }
        else if (rightTrigger > 0) {

            motorSpeed    = rightTrigger;
            povMotorSpeed = 0;
        }
        else {

            // No triggers are pressed, use the POV to control the motor speed
            if (pov == 0) {

                povMotorSpeed += 0.004;

                if (povMotorSpeed > 1.0) {
                    povMotorSpeed = 1.0;
                }
            }

            if (pov == 180) {

                povMotorSpeed -= 0.004;

                if (povMotorSpeed < -1.0) {
                    povMotorSpeed = -1.0;
                }
            }

            motorSpeed = povMotorSpeed;
        }

        SmartDashboard.putNumber("Test Motor Speed", motorSpeed);

        /*
         * If the X button is pressed, reset the motor speed to zero
         */

        if (controller.getXButton()) {
            motorSpeed    = 0;
            povMotorSpeed = 0;
        }

        /*
         * Apply the motor speed to the selected motor
         */

        SmartDashboard.putNumber("Test Motor Speed", motorSpeed);

        switch (selectedMotor) {

        case NONE:
            break;

        case DRIVE_LEFT_1:
            driveSubsystem.setTestMotorSpeeds(motorSpeed, 0, 0, 0);
            SmartDashboard.putBoolean("Test Left", true);
            break;

        case DRIVE_LEFT_2:
            driveSubsystem.setTestMotorSpeeds(0, motorSpeed, 0, 0);
            SmartDashboard.putBoolean("Test Left", true);
            break;

        case DRIVE_LEFT:
            driveSubsystem.setMotorSpeeds(motorSpeed, 0);
            SmartDashboard.putBoolean("Test Left", true);
            break;

        case DRIVE_RIGHT_1:
            driveSubsystem.setTestMotorSpeeds(0, 0, motorSpeed, 0);
            SmartDashboard.putBoolean("Test Right", true);
            break;

        case DRIVE_RIGHT_2:
            driveSubsystem.setTestMotorSpeeds(0, 0, 0, motorSpeed);
            SmartDashboard.putBoolean("Test Right", true);
            break;

        case DRIVE_RIGHT:
            driveSubsystem.setMotorSpeeds(0, motorSpeed);
            SmartDashboard.putBoolean("Test Right", true);
            break;

        case ARM_LIFT_1:
            if (motorSpeed != 0) {
                armSubsystem.setArmLiftPidEnabled(false);
                armSubsystem.setArmLiftTestSpeed(motorSpeed, 0);
            }
            SmartDashboard.putBoolean("Test Arm", true);
            break;

        case ARM_LIFT_2:
            if (motorSpeed != 0) {
                armSubsystem.setArmLiftPidEnabled(false);
                armSubsystem.setArmLiftTestSpeed(0, motorSpeed);
            }
            SmartDashboard.putBoolean("Test Arm", true);
            break;

        case ARM_LIFT:
            armSubsystem.setArmLiftPidEnabled(false);
            armSubsystem.setArmLiftSpeed(motorSpeed);
            SmartDashboard.putBoolean("Test Arm", true);
            break;

        case ARM_LIFT_PID:
            armSubsystem.setArmLiftPidEnabled(true);
            armSubsystem.moveArmLiftToAngle(ArmConstants.ARM_DOWN_ANGLE_DEGREES + motorSpeed * 100);
            SmartDashboard.putBoolean("Test Arm", true);
            break;

        case ARM_EXTEND:
            armSubsystem.setArmExtendSpeed(motorSpeed);
            SmartDashboard.putBoolean("Test Extend", true);
            break;

        case PINCHER:
            armSubsystem.setPincherSpeed(motorSpeed);
            SmartDashboard.putBoolean("Test Pincher", true);
            break;

        case CAMERA:
            // Use the motor speed to position the camera angle
            visionSubsystem.setCameraMotorSpeed(motorSpeed);
            SmartDashboard.putBoolean("Test Camera", true);
            break;
        }
    }

    @Override
    public boolean isFinished() {

        // Wait 1/2 second before finishing.
        // This allows the user to start this command using the start and back
        // button combination without cancelling on the start button as
        // the user releases the buttons
        if (System.currentTimeMillis() - startTime < 500) {
            return false;
        }

        // Cancel on the regular cancel button after the first 0.5 seconds
        if (driverController.isCancel()) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        System.out.println("SystemTestCommand : "
            + (interrupted ? "interrupted" : " cancelled by user"));

        stopAllMotors();

        selectedMotor = Motor.NONE;

        SmartDashboard.putBoolean("Test Mode", false);
        SmartDashboard.putString("Test Motor", selectedMotor.toString());
        SmartDashboard.putNumber("Test Motor Speed", 0);

    }

    private void clearMotorIndicators() {
        SmartDashboard.putBoolean("Test Left", false);
        SmartDashboard.putBoolean("Test Right", false);
        SmartDashboard.putBoolean("Test Arm", false);
        SmartDashboard.putBoolean("Test Extend", false);
        SmartDashboard.putBoolean("Test Pincher", false);
        SmartDashboard.putBoolean("Test Camera", false);
    }

    // Safely stop all motors in all subsystems used by this command.
    private void stopAllMotors() {

        driveSubsystem.stop();
        armSubsystem.stop();
        visionSubsystem.stop();

        povMotorSpeed = 0;
    }
}
