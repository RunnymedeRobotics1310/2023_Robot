package frc.robot.commands.auto.oldHumberCommands;

import static frc.robot.Constants.ArmConstants.MAX_EXTEND_SPEED;
import static frc.robot.Constants.ArmConstants.MAX_PINCHER_SPEED;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

abstract class HA_BaseArmCommand extends Command {

    protected final ArmSubsystem armSubsystem;

    protected HA_BaseArmCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    protected final void printArmState() {
        System.out.println("BaseArmCommand: armSubsystem.getHeldGamePiece: " + armSubsystem.getHeldGamePiece());
        System.out.println("BaseArmCommand: Arm angle: " + armSubsystem.getArmLiftAngle());
        System.out.println("BaseArmCommand: Arm extent: " + armSubsystem.getArmExtendEncoder());
        System.out.println("BaseArmCommand: Pincher: " + armSubsystem.getPincherEncoder());
    }

    /**
     * Convenience method to move the arm to the bottom position (lowest arm angle).
     *
     * @return {@code true} if at the bottom, {@code false} otherwise
     */
    protected boolean lowerArmToBottom() {
        return moveArmLiftToAngle(ArmConstants.ARM_DOWN_ANGLE_DEGREES);
    }

    /**
     * Move the motor to a specified encoder count
     *
     * @param targetAngle the angle in degrees
     * @return true if at the desired location, false if still moving to that point
     */
    protected final boolean moveArmLiftToAngle(double targetAngle) {
        armSubsystem.moveArmLiftToAngle(targetAngle);
        return armSubsystem.isArmAtLiftAngle(targetAngle);
    }

    /**
     * Convenience method to fully retract the arm.
     *
     * @return {@code true} if in frame, {@code false} otherwise
     */
    protected boolean retractArm() {
        return moveArmExtendToEncoderCount(0, MAX_EXTEND_SPEED);
    }

    /**
     * Move the motor to a specified encoder count
     *
     * @param targetCount the count to get to
     * @param speed the absolute value of the speed at which you should move
     * @return true if at the desired location, false if still moving to that point
     */
    protected final boolean moveArmExtendToEncoderCount(double targetCount, double speed) {
        speed = Math.abs(speed); // ignore negative values

        SmartDashboard.putBoolean("At Extension Position", armSubsystem.isAtExtendPosition(targetCount));

        // Special logic for fully retracting (ignore the encoder).
        if (targetCount <= 0) {

            if (armSubsystem.isArmRetracted()) {
                armSubsystem.setArmExtendSpeed(0);
                return true;
            }

            if (armSubsystem.getArmExtendEncoder() < ArmConstants.ARM_EXTEND_SLOW_ZONE_ENCODER_VALUE
                && speed > ArmConstants.MAX_EXTEND_SLOW_ZONE_SPEED) {

                speed = ArmConstants.MAX_EXTEND_SLOW_ZONE_SPEED;
            }

            armSubsystem.setArmExtendSpeed(-speed);

            return false;
        }

        // Logic when the arm is not being fully retracted.
        if (armSubsystem.isAtExtendPosition(targetCount)) {
            armSubsystem.setArmExtendSpeed(0);
            return true;
        }

        double gap    = targetCount - armSubsystem.getArmExtendEncoder();

        // Determine whether to slow down because we are close to the target.
        if (Math.abs(gap) < ArmConstants.ARM_EXTEND_SLOW_ZONE_ENCODER_VALUE) {

            speed = Math.min(speed, ArmConstants.MAX_EXTEND_SLOW_ZONE_SPEED);
        }

        armSubsystem.setArmExtendSpeed(gap < 0 ? -speed : speed);

        return false;
    }

    /**
     * Convenience method to set the pincher inside the frame for storage.
     *
     * @return {@code true} if in frame, {@code false} otherwise
     */
    protected boolean movePincherInsideFrame() {
        return movePincherToEncoderCount(ArmConstants.MIN_PINCHER_INSIDE_FRAME_POSITION);
    }

    /**
     * Move the pincher motor to a specified encoder count
     *
     * @param targetCount the count to get to
     * @return true if at the desired location, false if still moving to that point
     */
    protected final boolean movePincherToEncoderCount(double targetCount) {

        SmartDashboard.putBoolean("At Pincher Position", armSubsystem.isAtPincherPosition(targetCount));

        double gap   = targetCount - armSubsystem.getPincherEncoder();
        double speed = MAX_PINCHER_SPEED;

        /*
         * If the target count is zero, then move the pincher until the open limit
         * is detected. Assume that the encoder counts are not correct - the encoder
         * could read a negative value.
         */

        if (targetCount == 0) {

            // Check if the limit is reached
            if (armSubsystem.isPincherOpen()) {
                armSubsystem.setPincherSpeed(0);
                return true;
            }

            // When closing to the limit, slow down when the encoders read a value close to the limit.
            // Note: if the encoders are way off, then it may take a while to get to the
            // limit from here which could affect the auto.
            if (gap > -ArmConstants.PINCHER_SLOW_ZONE_ENCODER_VALUE) {
                speed = ArmConstants.MAX_PINCHER_SLOW_ZONE_SPEED;
            }

            // Close the pincher at the required speed.
            armSubsystem.setPincherSpeed(-speed);
        }
        else {

            // Check if the position is reached
            if (armSubsystem.isAtPincherPosition(targetCount)) {
                armSubsystem.setPincherSpeed(0);
                return true;
            }

            // Determine whether to slow down because we are close to the target.
            if (Math.abs(gap) < ArmConstants.PINCHER_SLOW_ZONE_ENCODER_VALUE) {

                // If opening (-ve gap), then always slow down when close to the target
                if (gap < 0) {
                    speed = ArmConstants.MAX_PINCHER_SLOW_ZONE_SPEED;
                }
                else {
                    // If closing (+ve gap) and no game piece is detected, then slow down when
                    // close to the target.
                    // If a game piece is detected, then ignore the slowing
                    // in order to get a good grip on the game piece.
                    if (!armSubsystem.isGamePieceDetected()) {
                        speed = ArmConstants.MAX_PINCHER_SLOW_ZONE_SPEED;
                    }
                }

            }

            // Set the pincher speed to the calculated speed based on the gap
            armSubsystem.setPincherSpeed(gap < 0 ? -speed : speed);
        }

        return false;
    }

    /**
     * Move the motor to a specified encoder count
     *
     * @return true if at the desired location, false if still moving to that point
     */
    protected final boolean openPincher() {

        return movePincherToEncoderCount(0);
    }

    protected final void stopArmMotors() {
        armSubsystem.stop();
    }


    protected final boolean isCompactPose() {
        return armSubsystem.isArmDown()
            && armSubsystem.isArmRetracted() && armSubsystem.isPincherInsideFrame();
    }


}

