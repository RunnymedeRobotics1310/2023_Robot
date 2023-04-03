package frc.robot.commands.arm;

import frc.robot.Constants.GameConstants.ScoringRow;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.Constants.GameConstants.GamePiece.NONE;
import static frc.robot.Constants.GameConstants.GamePiece.CONE;
import static frc.robot.Constants.GameConstants.GamePiece.CUBE;

public class AutoTuneScore extends BaseArmCommand {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private GamePiece heldGamePiece = NONE;

    private double visionTargetHeadingError = 0;

    private double visionTargetHeightError = 0;

    private double visionTargetExtendError = 0;

    private double ultrasonicDistanceCm = 0;

    // fixme; once these are fully established, move them to Constants in a reasonable location
    /**
     * Exactly how far off the target angle offset from the vision subsystem we want to be when perfectly aligned
     */
    private static final double SCORE_CONE_HIGH_IDEAL_HEADING_ERROR = 0;
    /**
     * Exactly how far from the target height we want the arm to be raised to, in order to
     * be perfectly aligned
     */
    private static final double SCORE_CONE_HIGH_IDEAL_TARGET_HEIGHT_ERROR = 0;
    /**
     * Exactly how much difference between the arm extension and the distance from vision
     * we want to be
     */
    private static final double SCORE_CONE_HIGH_IDEAL_EXTEND_ERROR = 0;
    private static final double SCORE_CONE_HIGH_IDEAL_ULTRASONIC_DISTANCE_ERROR = 0;

    public AutoTuneScore(ScoringRow scoringRow, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
        super(armSubsystem);
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem, visionSubsystem); // arm subsystem is automatically added as a requirement
    }

    @Override
    public void initialize() {
        super.initialize();
        heldGamePiece = armSubsystem.getHeldGamePiece();
        if (heldGamePiece == CONE) {
            visionTargetHeadingError = visionSubsystem.getTargetAngleOffset();
            visionTargetHeightError = visionSubsystem.getTargetVerticalOffset();
            visionTargetExtendError = visionSubsystem.getTargetDistance();
            ultrasonicDistanceCm    = driveSubsystem.getUltrasonicDistanceCm();
        }
    }

    @Override
    public void execute() {
        switch (heldGamePiece) {
        case CONE:
            // fixme: align wheels - back up to correct, drive forward to correct
            // fixme: align height - if not in a suitable scoring position, raise/lower arm
            // fixme: align extend - if not in suitable position, extend a bit
            break;
        case CUBE:
            setFinishReason("cannot auto-tune positioning for cube");
            break;
        case NONE:
            setFinishReason("no game piece detected");
            break;
        default:
            setFinishReason("invalid game piece: "+heldGamePiece);
            break;
    }
    }

    @Override
    public boolean isFinished() {
        if (heldGamePiece == CUBE) {
            return true;
        }

    }
}
