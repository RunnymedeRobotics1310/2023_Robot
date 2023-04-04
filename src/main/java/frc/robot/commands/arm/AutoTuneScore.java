package frc.robot.commands.arm;

import frc.robot.Constants;
import frc.robot.Constants.GameConstants.ScoringRow;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.Constants.GameConstants.GamePiece.NONE;

public class AutoTuneScore extends BaseArmCommand {
    private final DriveSubsystem  driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private ScoringRow scoringRow    = ScoringRow.TOP;
    private GamePiece  heldGamePiece = NONE;

    private double visionTargetHeadingAngleError = 0;

    private double visionTargetVerticalAngleError = 0;

    private double visionTargetDistanceError = 0;


    private static final double SCORE_CONE_HIGH_MAX_HEADING_ERROR       = 1; // fixme: correct the value
    private static final double SCORE_CONE_HIGH_TARGET_VISION_ANGLE     = 32; // fixme: measure and set
    private static final double SCORE_CONE_HIGH_MAX_TARGET_HEIGHT_ERROR = 1; // fixme: correct the value
    private static final double SCORE_CONE_HIGH_TARGET_DISTANCE         = 99; // fixme: measure and set
    private static final double SCORE_CONE_HIGH_MAX_EXTEND_ERROR        = 1; // fixme: correct the value

    public AutoTuneScore(ScoringRow scoringRow, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem,
        DriveSubsystem driveSubsystem) {
        super(armSubsystem);
        this.driveSubsystem  = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem, visionSubsystem); // arm subsystem is automatically added as a requirement
        this.scoringRow = scoringRow;
    }

    @Override
    public void initialize() {
        super.initialize();
        heldGamePiece                  = armSubsystem.getHeldGamePiece();
        visionTargetHeadingAngleError  = 0;
        visionTargetVerticalAngleError = 0;
        visionTargetDistanceError      = 0;
    }

    @Override
    public void execute() {
        align();
        adjustHeight();
        adjustExtension();
    }

    @Override
    public boolean isFinished() {
        boolean tuned = isAligned() && isHeightOk() && isExtendOk();
        setFinishReason("Robot in position to score");
        return tuned;
    }

    private void align() {

        // compute the error
        switch (heldGamePiece) {
        case CONE:
            // any offset is bad
            visionTargetHeadingAngleError = visionSubsystem.getTargetAngleOffset();
            break;
        case CUBE:
        default:
            return;
        }

        // fix if out of alignment
        if (!isAligned()) {
            final double ADJUST_DRIVE_SPEED = 0.01;
            double       turn               = visionTargetHeadingAngleError * .01;
            double       leftSpeed          = ADJUST_DRIVE_SPEED + turn;
            double       rightSpeed         = ADJUST_DRIVE_SPEED - turn;
            // NOTE: the robot may already be pushed right up against the grid
            driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
        }
        else {
            driveSubsystem.stop();
        }
    }

    private boolean isAligned() {
        switch (heldGamePiece) {
        case CONE:
            return Math.abs(visionTargetHeadingAngleError) > SCORE_CONE_HIGH_MAX_HEADING_ERROR;
        case CUBE:
        default:
            return true;
        }
    }

    private void adjustHeight() {
        // compute error
        switch (heldGamePiece) {
        case CONE:
            visionTargetVerticalAngleError = SCORE_CONE_HIGH_TARGET_VISION_ANGLE - visionSubsystem.getTargetVerticalAngleOffset();
            break;
        case CUBE:
        default:
            visionTargetVerticalAngleError = 0;
        }

        // fix if incorrect
        if (!isHeightOk()) {
            // move arm one degree at a time to correct angle (note vision angle is not the same as arm angle)
            double correction = Math.signum(visionTargetVerticalAngleError);
            armSubsystem.moveArmLiftToAngle(armSubsystem.getArmLiftAngle() + correction);
        }
    }

    private boolean isHeightOk() {

        switch (heldGamePiece) {
        case CONE:
            return Math.abs(visionTargetVerticalAngleError) > SCORE_CONE_HIGH_MAX_TARGET_HEIGHT_ERROR;
        case CUBE:
        default:
            return true;
        }
    }

    private void adjustExtension() {
        // compute error
        switch (heldGamePiece) {
        case CONE:
            visionTargetDistanceError = SCORE_CONE_HIGH_TARGET_DISTANCE - visionSubsystem.getTargetDistanceCm();
            break;
        case CUBE:
        default:
            visionTargetDistanceError = 0;
        }
        // fix if incorrect
        if (!isExtendOk()) {
            // move extend one encoder count at a time until it is in position
            double correction = Math.signum(visionTargetDistanceError);
            moveArmExtendToEncoderCount(armSubsystem.getArmExtendEncoder() + correction,
                Constants.ArmConstants.MAX_LIFT_SLOW_ZONE_SPEED);
        }
    }

    private boolean isExtendOk() {
        switch (heldGamePiece) {
        case CONE:
            return Math.abs(visionTargetDistanceError) > SCORE_CONE_HIGH_MAX_EXTEND_ERROR;
        case CUBE:
        default:
            return true;
        }
    }
}
