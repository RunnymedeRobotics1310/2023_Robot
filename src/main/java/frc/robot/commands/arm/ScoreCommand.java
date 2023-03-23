package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.ScoringRow;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.GameConstants.GamePiece.NONE;

public class ScoreCommand extends BaseArmCommand {

    private final long       MAX_TIME_TO_SCORE_MILLIS = 4000L;

    private final ScoringRow scoringRow;
    private ArmPosition      scoringPosition          = null;
    private GamePiece        gamePiece                = null;

    private enum Step {
        RETRACT, LIFT_TO_CLEAR, LIFT_AND_EXTEND, FINISH
    }

    private Step step              = null;
    private long commandStartTime  = 0;

    public ScoreCommand(ScoringRow scoringRow, ArmSubsystem armSubsystem) {
        super(armSubsystem);
        this.scoringRow = scoringRow;
    }

    @Override
    public void initialize() {

        commandStartTime = System.currentTimeMillis();

        gamePiece       = armSubsystem.getHeldGamePiece();
        scoringPosition = ArmConstants.getScoringPosition(gamePiece, scoringRow);

        double armAngle = armSubsystem.getArmLiftAngle();
        double armExtent = armSubsystem.getArmExtendEncoder();
        if (gamePiece == NONE) {
            logCommandStart("Scoring row : " + scoringRow + ", cancelling - no held game piece");
            this.cancel();
        } else if (armAngle >= ArmConstants.CLEAR_FRAME_ARM_ANGLE) {
            step = Step.LIFT_AND_EXTEND;
            logCommandStart("Scoring row : " + scoringRow + ", Starting Step " + step + " (arm above clear frame angle)");
        } else if (armExtent >= ArmConstants.MAX_ARM_EXTEND_INSIDE_FRAME) {
            step = Step.LIFT_TO_CLEAR;
            logCommandStart("Scoring row : " + scoringRow + ", Starting Step " + step + " (arm already extended beyond frame but angle low)");
        } else {
            step = Step.RETRACT;
            logCommandStart("Scoring row : " + scoringRow + ", Starting Step " + step + " (arm in unsafe position)");
        }
    }

    @Override
    public void execute() {

        switch (step) {

        case RETRACT:

            // Ensure the arm is retracted before lifting
            if (!moveArmExtendToEncoderCount(0, 1)) {
                return;
            }

            logStateTransition("RETRACT -> LIFT_TO_CLEAR");

            step = Step.LIFT_TO_CLEAR;
            return;

        case LIFT_TO_CLEAR:

            // Lift the arm until in position
            if (moveArmLiftToAngle(ArmConstants.CLEAR_FRAME_ARM_ANGLE)) {
                step = Step.LIFT_AND_EXTEND;
                logStateTransition("LIFT_TO_CLEAR -> LIFT_AND_EXTEND");

            }
            return;

        case LIFT_AND_EXTEND:

            boolean angleGood = moveArmLiftToAngle(scoringPosition.angle);
            boolean extendGood = moveArmExtendToEncoderCount(scoringPosition.extension, .7);
            if (angleGood && extendGood) {
                step = Step.FINISH;

                logStateTransition("LIFT_AND_EXTEND -> FINISH");

                stopArmMotors();

            }
            return;

        default:
        }

    }

    @Override
    public boolean isFinished() {

        if (step == Step.FINISH) {
            setFinishReason("Arm in position");
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
