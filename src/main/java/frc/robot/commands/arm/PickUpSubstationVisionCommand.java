package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.VisionConstants.CameraView;
import frc.robot.Constants.VisionConstants.VisionTarget;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.commands.vision.SetVisionTargetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class PickUpSubstationVisionCommand extends BaseArmCommand {

    private static final double   MIN_PICKUP_DISTANCE             = 95;
    private static final double   MAX_PICKUP_DISTANCE             = 115;
    private static final double   TARGET_CAMERA_OFFSET            = 5;
    private static final double   VISION_TARGET_HEADING_TOLERANCE = 1;

    private final VisionSubsystem visionSubsystem;
    private final DriveSubsystem  driveSubsystem;
    private final OperatorInput   operatorInput;

    private ArmPosition           shelfTargetArmPosition          = null;
    private double                pauseStartTime                  = 0;
    private double                alignStartTime                  = 0;

    /**
     * Only cone is supported for now.
     */
    private GamePiece             gamePiece                       = GamePiece.CONE;

    private double                visionTargetHeadingError        = 0;

    private enum State {
        GET_WITHIN_RANGE, ALIGN, PAUSE, EXTEND_ARM, PICKUP
    }

    private State currentState = State.GET_WITHIN_RANGE;

    public PickUpSubstationVisionCommand(OperatorInput operatorInput, ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem,
        VisionSubsystem visionSubsystem) {

        super(armSubsystem);

        this.visionSubsystem = visionSubsystem;
        this.driveSubsystem  = driveSubsystem;
        this.operatorInput   = operatorInput;

        addRequirements(driveSubsystem);
        // FIXME: Add visionSubsystem as requirement? Most likely mandatory.
    }

    @Override
    public void initialize() {

        // Schedule to set up the camera in parallel
        CommandScheduler.getInstance().schedule(
            new SetVisionTargetCommand(VisionTarget.CONE_SUBSTATION, visionSubsystem));

        visionTargetHeadingError = 0;

        stopArmMotors();

        currentState = State.GET_WITHIN_RANGE;

        logCommandStart("Starting state: GET_WITHIN_RANGE");
    }

    @Override
    public void execute() {

        double armAngle             = armSubsystem.getArmLiftAngle();
        double armExtendPosition    = armSubsystem.getArmExtendEncoder();
        double ultrasonicDistanceCm = driveSubsystem.getUltrasonicDistanceCm();

        switch (currentState) {

        case GET_WITHIN_RANGE: {

            double speed = calcSpeedToScoringRange();
            driveSubsystem.setMotorSpeeds(speed, speed);

            // Wait for the camera to get into position
            if (visionSubsystem.getCameraView() == CameraView.HIGH) {

                currentState = State.ALIGN;
                logStateTransition("GET_WITHIN_RANGE -> ALIGN");

                alignStartTime = System.currentTimeMillis();

                // If there is a vision target, then update the error
                if (visionSubsystem.isVisionTargetFound()) {
                    visionTargetHeadingError = visionSubsystem.getTargetAngleOffset();
                    System.out.print("target aquired");
                }
            }

            return;
        }

        case ALIGN: {

            // If there is a vision target, then update the error
            if (visionSubsystem.isVisionTargetFound()) {
                visionTargetHeadingError = visionSubsystem.getTargetAngleOffset();
            }

            if (System.currentTimeMillis() - alignStartTime > 1000) {
                currentState = State.PAUSE;
                // Stop the motors
                driveSubsystem.setMotorSpeeds(0, 0);
                pauseStartTime = System.currentTimeMillis();

                logStateTransition("ALIGN -> PAUSE",
                    "Timing out alignment prior to aligning perfectly. Current error: " + visionTargetHeadingError);
                return;
            }

            double speed = calcSpeedToScoringRange();

            alignToVisionTarget(speed);

            if (speed == 0
                && Math.abs(visionTargetHeadingError) < 2) {

                // Stop the motors
                driveSubsystem.setMotorSpeeds(0, 0);

                pauseStartTime = System.currentTimeMillis();

                currentState   = State.PAUSE;
                logStateTransition("ALIGN -> PAUSE", "Aligned, Vision Target Error " + visionTargetHeadingError);
            }

            // Wait for target and distance alignment
            return;

        }

        case PAUSE:

            // Stop the motors
            driveSubsystem.setMotorSpeeds(0, 0);

            // Wait 250 ms for the base to stop moving and for the ultrasonic to settle.
            if (System.currentTimeMillis() - pauseStartTime > 250) {

                // Set the required extension distance based on the following formula
                double requiredExtensionEncoderPosition = ultrasonicDistanceCm * .88 - 53.1;

                double pickupAngle                      = ArmConstants.SUBSTATION_PICKUP_POSITION.angle;

                // tweak the angle slightly higher if closer to the substation. The range of
                // distances is 75-115 cm.
                // NOTE: This calculation must be done before raising the arm in the way.

                pickupAngle            += (115 - ultrasonicDistanceCm) / 12d;

                shelfTargetArmPosition  = new ArmPosition(pickupAngle, requiredExtensionEncoderPosition);

                currentState            = State.EXTEND_ARM;

                logStateTransition("PAUSE -> EXTEND_ARM", "Target arm position " + shelfTargetArmPosition);
            }

            return;


        case EXTEND_ARM:

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

            if (moveArmLiftToAngle(shelfTargetArmPosition.angle)) {

                if (moveArmExtendToEncoderCount(shelfTargetArmPosition.extension, ArmConstants.MAX_EXTEND_SPEED)) {

                    currentState = State.PICKUP;
                    logStateTransition("EXTEND_ARM -> PICKUP");
                }
            }

            return;

        case PICKUP:

            movePincherToEncoderCount(gamePiece.pincherEncoderCount);

            return;

        default:
            return;
        }

    }

    @Override
    public boolean isFinished() {

        // If holding the required piece
        if (armSubsystem.getHeldGamePiece() == gamePiece) {
            setFinishReason("Game Piece held " + gamePiece);
            return true;
        }

        // If the pincher is closed the command ends even if there is no game piece
        if (armSubsystem.isAtPincherPosition(gamePiece.pincherEncoderCount)) {
            setFinishReason("At pincher close position");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        stopArmMotors();

        logCommandEnd(interrupted);

        // In Teleop, pick the next command
        if (DriverStation.isTeleopEnabled()) {

            CommandScheduler.getInstance().schedule(new PickupGamePieceCommand(gamePiece, operatorInput, armSubsystem));
        }
    }

    private double calcSpeedToScoringRange() {

        // Move forward or backwards until the distance is in range
        // Scoring can only happen between 75 and 115
        double driveSpeed = 0;

        if (driveSubsystem.getUltrasonicDistanceCm() < MIN_PICKUP_DISTANCE) {
            driveSpeed = -.2;
        }

        if (driveSubsystem.getUltrasonicDistanceCm() > MAX_PICKUP_DISTANCE) {
            driveSpeed = .2;
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

        if (Math.abs(visionTargetHeadingError) < VISION_TARGET_HEADING_TOLERANCE) {
            turn = 0;
        }

        double leftSpeed = driveSpeed + turn;
        double rightSpeed = driveSpeed - turn;
        log("alignToVisionTarget leftSpeed: "+(Math.round(leftSpeed*1000)/1000d)+" rightSpeed:"+(Math.round(rightSpeed*1000)/1000d));
        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);

        return turn == 0;
    }

}
