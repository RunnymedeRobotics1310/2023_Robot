package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.VisionConstants.CameraView;
import frc.robot.commands.vision.SetCameraViewCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.VisionSubsystem.VisionTargetType;

public class PickUpSubstationVisionCommand extends BaseArmCommand {

    private static final double   MIN_PICKUP_DISTANCE             = 95;
    private static final double   MAX_PICKUP_DISTANCE             = 115;
    private static final double   TARGET_CAMERA_OFFSET            = 5;
    private static final double   VISION_TARGET_HEADING_TOLERANCE = 1;

    private final VisionSubsystem visionSubsystem;
    private final DriveSubsystem  driveSubsystem;

    private double                requiredExtensionEncoderPosition;

    private GamePiece             gamePiece                       = GamePiece.CONE;

    private double                visionTargetHeadingError        = 0;

    private enum State {
        MOVE_CAMERA_AND_GET_WITHIN_RANGE, ALIGN, PICKUP
    }

    private State currentState = State.MOVE_CAMERA_AND_GET_WITHIN_RANGE;

    public PickUpSubstationVisionCommand(ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem,
        VisionSubsystem visionSubsystem) {

        super(armSubsystem);

        this.visionSubsystem = visionSubsystem;
        this.driveSubsystem  = driveSubsystem;

        addRequirements(driveSubsystem);
        requiredExtensionEncoderPosition = 0;
    }

    @Override
    public void initialize() {

        System.out.println("PickUp Substation started.  GamePiece " + gamePiece);

        visionSubsystem.setVisionTargetType(VisionTargetType.CONE);
        moveCameraToHighPosition();

        visionTargetHeadingError = 0;

        printArmState();
        stopArmMotors();

        currentState = State.MOVE_CAMERA_AND_GET_WITHIN_RANGE;
    }

    @Override
    public void execute() {

        double armAngle             = armSubsystem.getArmLiftAngle();
        double armExtendPosition    = armSubsystem.getArmExtendEncoder();
        double ultrasonicDistanceCm = driveSubsystem.getUltrasonicDistanceCm();

        switch (currentState) {

        case MOVE_CAMERA_AND_GET_WITHIN_RANGE: {

            moveToScoringRange();

            // Wait for the camera to get into position
            if (visionSubsystem.getCameraView() == CameraView.HIGH) {

                currentState = State.ALIGN;

                // If there is a vision target, then update the error
                if (visionSubsystem.isVisionTargetFound()) {
                    visionTargetHeadingError = visionSubsystem.getTargetAngleOffset();
                }
            }

            return;
        }

        case ALIGN: {

            // If there is a vision target, then update the error
            if (visionSubsystem.isVisionTargetFound()) {
                visionTargetHeadingError = visionSubsystem.getTargetAngleOffset();
            }

            double speed = moveToScoringRange();

            alignToVisionTarget(speed);

            if (speed == 0
                && Math.abs(visionTargetHeadingError) < 2) {

                // Stop the motors
                driveSubsystem.setMotorSpeeds(0, 0);

                // Set the required extension distance based on the following formula
                requiredExtensionEncoderPosition = ultrasonicDistanceCm * .88 - 53.1;

                currentState                     = State.PICKUP;
            }

            // Wait for target and distance alignment
            return;

        }

        case PICKUP:

            /*
             * Special logic to make sure the arm comes up over the frame when extending.
             */
            if (armAngle < (ArmConstants.CLEAR_FRAME_ARM_ANGLE - ArmConstants.ARM_LIFT_ANGLE_TOLERANCE_DEGREES)
                && armExtendPosition < ArmConstants.MAX_ARM_EXTEND_INSIDE_FRAME) {

                // Retract the arm before lifting.
                if (!retractArm()) {
                    return; // Wait for the retraction before lifting the arm
                }

                moveArmLiftToAngle(ArmConstants.CLEAR_FRAME_ARM_ANGLE + ArmConstants.ARM_LIFT_ANGLE_TOLERANCE_DEGREES + 2);
                return;
            }

            /*
             * Move the arm to set up for an intake from the substation.
             */

            // Always open the pincher, there is no point in waiting
            openPincher();

            if (moveArmLiftToAngle(ArmConstants.SUBSTATION_PICKUP_POSITION.angle)) {


                if (moveArmExtendToEncoderCount(requiredExtensionEncoderPosition, ArmConstants.MAX_EXTEND_SPEED)) {

                    if (armSubsystem.isGamePieceDetected()) {
                        movePincherToEncoderCount(gamePiece.pincherEncoderCount);
                    }
                }
            }
        }

    }

    @Override
    public boolean isFinished() {

        // If holding the required piece
        if (armSubsystem.getHeldGamePiece() == gamePiece) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        stopArmMotors();

        if (interrupted) {
            System.out.print("Pick Up Substation interrupted");
        }
        else {
            System.out.print("Pick Up Substation ended");
        }
        printArmState();

        // In Teleop, pick the next command
        if (DriverStation.isTeleopEnabled()) {

            if (armSubsystem.isGamePieceDetected()) {
                CommandScheduler.getInstance().schedule(new PickupGamePieceCommand(gamePiece, armSubsystem));
            }
            else {
                CommandScheduler.getInstance().schedule(new CompactCommand(armSubsystem));
            }

        }
    }

    private double moveToScoringRange() {

        // Move forward or backwards until the distance is in range
        // Scoring can only happen between 75 and 115
        double driveSpeed = 0;

        if (driveSubsystem.getUltrasonicDistanceCm() < MIN_PICKUP_DISTANCE) {
            driveSpeed = -.1;
        }

        if (driveSubsystem.getUltrasonicDistanceCm() > MAX_PICKUP_DISTANCE) {
            driveSpeed = .1;
        }

        // If there is no forward or backward movement, then the
        // robot is within scoring range.
        return driveSpeed;
    }

    private boolean alignToVisionTarget(double driveSpeed) {

        // Update the vision target error only when the target is found.
        if (visionSubsystem.isVisionTargetFound()) {
            visionTargetHeadingError = visionSubsystem.getTargetAngleOffset() - TARGET_CAMERA_OFFSET;
        }

        double turn = visionTargetHeadingError * .01;

        if (visionTargetHeadingError < VISION_TARGET_HEADING_TOLERANCE) {
            turn = 0;
        }

        driveSubsystem.setMotorSpeeds(driveSpeed + turn, driveSpeed - turn);

        return turn == 0;
    }

    private void moveCameraToHighPosition() {

        if (DriverStation.isAutonomousEnabled() || DriverStation.isTeleopEnabled()) {

            CommandScheduler.getInstance().schedule(new SetCameraViewCommand(CameraView.HIGH, visionSubsystem));
        }
    }
}
