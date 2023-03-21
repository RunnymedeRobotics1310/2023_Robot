package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.ScoringRow;
import frc.robot.subsystems.ArmSubsystem;

public class ScoreAutoCommand extends BaseArmCommand {

    private final long       MAX_TIME_TO_SCORE_MILLIS = 4000L;

    private final ScoringRow scoringRow;
    private ArmPosition      scoringPosition          = null;
    private GamePiece        gamePiece                = null;

    private enum Step {
        RETRACT, LIFT_AND_EXTEND, LOWER_ARM, FINISH
    }

    private Step step              = null;
    private long commandStartTime  = 0;
    private long lowerArmStartTime = 0;

    public ScoreAutoCommand(ScoringRow scoringRow, ArmSubsystem armSubsystem) {
        super(armSubsystem);
        this.scoringRow = scoringRow;
    }

    @Override
    public void initialize() {

        gamePiece       = armSubsystem.getHeldGamePiece();
        scoringPosition = ArmConstants.getScoringPosition(gamePiece, scoringRow);

        stopArmMotors();

        commandStartTime = System.currentTimeMillis();
        step             = Step.RETRACT;

        logCommandStart("Scoring row : " + scoringRow + ", Starting Step " + step);
    }

    @Override
    public void execute() {

        switch (step) {

        case RETRACT:

            // Ensure the arm is retracted before lifting
            if (!moveArmExtendToEncoderCount(0, 1)) {
                return;
            }

            logStateTransition("RETRACT -> LIFT_AND_EXTEND");

            step = Step.LIFT_AND_EXTEND;
            return;

        case LIFT_AND_EXTEND:

            // Lift the arm until in position
            if (moveArmLiftToAngle(scoringPosition.angle)) {

                // Then extend
                if (moveArmExtendToEncoderCount(scoringPosition.extension, .7)) {

                    step              = Step.LOWER_ARM;
                    lowerArmStartTime = System.currentTimeMillis();

                    logStateTransition("LIFT_AND_EXTEND -> LOWER_ARM");

                    stopArmMotors();
                    return;
                }
            }

            return;

        case LOWER_ARM:

            // Do not lower the arm if already on the lowest scoring position
            if (scoringRow == ScoringRow.BOTTOM) {

                logStateTransition("LOWER_ARM -> FINISH", "Target is bottom row");

                step = Step.FINISH;
                return;
            }

            // Lower the arm onto the post
            // Lower the arm by 5 degrees or .5 seconds whichever comes first.
            if (moveArmLiftToAngle(scoringPosition.angle - 11)) {
                logStateTransition("LOWER_ARM -> FINISH", "Arm lowered");
                step = Step.FINISH;
                return;
            }

            if (System.currentTimeMillis() - lowerArmStartTime > 500) {
                logStateTransition("LOWER_ARM -> FINISH", "Lower arm timed out after 500ms");
                step = Step.FINISH;
            }

            return;

        default:
            return;
        }

    }

    @Override
    public boolean isFinished() {

        if (step == Step.FINISH) {
            setFinishReason("Arm lowered");
            return true;
        }

        // If the command has been running for more than 3 seconds, then end.
        if (System.currentTimeMillis() - commandStartTime > MAX_TIME_TO_SCORE_MILLIS) {
            setFinishReason("Timed out after 3 seconds");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        stopArmMotors();

        logCommandEnd(interrupted);
    }
}
