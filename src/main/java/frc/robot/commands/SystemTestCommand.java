package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This command is used to safely stop the robot in its current position, and to
 * cancel any running commands
 */
public class SystemTestCommand extends CommandBase {

    private enum Motor {
        NONE, DRIVE_LEFT, DRIVE_RIGHT, ARM_LIFT, ARM_EXTEND, PINCHER
    };

    private final XboxController controller;
    private final DriveSubsystem driveSubsystem;
    private final ArmSubsystem   armSubsystem;

    private long                 startTime           = 0;
    private Motor                selectedMotor       = Motor.NONE;
    private double               motorSpeed          = 0;

    private boolean              previousLeftBumper  = false;
    private boolean              previousRightBumper = false;

    /**
     * Cancel the commands running on all subsystems.
     *
     * All subsystems must be passed to this command, and each subsystem should have a stop command
     * that safely stops the robot from moving.
     */
    public SystemTestCommand(XboxController controller, DriveSubsystem driveSubsystem,
        ArmSubsystem armSubsystem) {

        this.controller     = controller;
        this.driveSubsystem = driveSubsystem;
        this.armSubsystem   = armSubsystem;

        addRequirements(driveSubsystem, armSubsystem);
    }

    /**
     * The SystemTestCommand is not interruptible, and prevents all other commands that try to
     * interrupt it. Only the cancel button will end the SystemTestCommand.
     */
    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }

    @Override
    public void initialize() {

        System.out.println("SystemTestCommand : started.");

        stopAllMotors();

        startTime = System.currentTimeMillis();

        SmartDashboard.putBoolean("Test Mode", true);
        SmartDashboard.putString("Test Motor", selectedMotor.toString());
        SmartDashboard.putNumber("Test Motor Speed", motorSpeed);
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

                nextMotorIndex = (nextMotorIndex + 1) % Motor.values().length;
            }
            else {

                nextMotorIndex--;
                if (nextMotorIndex < 0) {
                    nextMotorIndex = Motor.values().length - 1;
                }
            }

            stopAllMotors();

            selectedMotor = Motor.values()[nextMotorIndex];

            SmartDashboard.putString("Test Motor", selectedMotor.toString());
        }

        /*
         * Once the motor is selected, use the dpad up and down to
         * adjust the motor speed.
         *
         * The speed is adjusted 50 times / second as the user holds the
         * POV control. Allow a 5 seconds to ramp the speed from 0 to full value.
         *
         * increment = 1.0 (full) / 50 adjustments/sec / 5 sec = .004 adjustment size / loop.
         */

        int pov = controller.getPOV();

        if (pov == 0) {

            motorSpeed += 0.04;

            if (motorSpeed > 1.0) {
                motorSpeed = 1.0;
            }
        }

        if (pov == 180) {

            motorSpeed -= 0.04;

            if (motorSpeed < -1.0) {
                motorSpeed = -1.0;
            }
        }

        /*
         * If the X button is pressed, reset the motor speed to zero
         */

        if (controller.getXButton()) {
            motorSpeed = 0;
        }

        /*
         * Apply the motor speed to the selected motor
         */

        SmartDashboard.putNumber("Test Motor Speed", motorSpeed);

        switch (selectedMotor) {

        case NONE:
            break;

        case DRIVE_LEFT:
            driveSubsystem.setMotorSpeeds(motorSpeed, 0);
            break;

        case DRIVE_RIGHT:
            driveSubsystem.setMotorSpeeds(0, motorSpeed);
            break;

        case ARM_LIFT:
            armSubsystem.setArmLiftSpeed(motorSpeed);
            break;

        case ARM_EXTEND:
            armSubsystem.setArmExtendSpeed(motorSpeed);
            break;

        case PINCHER:
            armSubsystem.setPincherSpeed(motorSpeed);
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
        if (controller.getStartButton()) {
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
        SmartDashboard.putNumber("Test Motor Speed", motorSpeed);

    }

    // Safely stop all motors in all subsystems used by this command.
    private void stopAllMotors() {

        driveSubsystem.stop();
        armSubsystem.stop();

        motorSpeed = 0;
    }
}