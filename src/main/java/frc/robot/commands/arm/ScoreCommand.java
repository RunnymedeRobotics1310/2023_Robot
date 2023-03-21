package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.ScoringRow;
import frc.robot.subsystems.ArmSubsystem;

public class ScoreCommand extends BaseArmCommand {

    private final long       MAX_TIME_TO_SCORE_MILLIS = 3000L;

    private final ScoringRow scoringRow;
    private ArmPosition      scoringPosition          = null;
    private GamePiece        gamePiece                = null;
    private long             startScoreTime           = 0L;

    public ScoreCommand(ScoringRow scoringRow, ArmSubsystem armSubsystem) {
        super(armSubsystem);
        this.scoringRow = scoringRow;
    }

    @Override
    public void initialize() {

        gamePiece       = armSubsystem.getHeldGamePiece();
        scoringPosition = ArmConstants.getScoringPosition(gamePiece, scoringRow);

        stopArmMotors();
        startScoreTime = System.currentTimeMillis();

        logCommandStart("Scoring row : " + scoringRow);
    }

    @Override
    public void execute() {

        if (!armSubsystem.isGamePieceDetected()) {
            return;
        }

        // If the arm is starting inside the frame, then
        // retract before moving the arm.
        if (armSubsystem.getArmLiftAngle() < ArmConstants.CLEAR_FRAME_ARM_ANGLE) {

            if (!moveArmExtendToEncoderCount(0, 1)) {
                return;
            }
        }

        // If higher than target, retract first
        if (armSubsystem.getArmLiftAngle() > scoringPosition.angle) {

            // Move the extender until it is in position
            if (moveArmExtendToEncoderCount(scoringPosition.extension, 1)) {

                // Then lift
                moveArmLiftToAngle(scoringPosition.angle);
            }

        }
        else {

            // Lift the arm until in position
            if (moveArmLiftToAngle(scoringPosition.angle)) {

                // Then extend
                moveArmExtendToEncoderCount(scoringPosition.extension, .5);
            }

        }
    }

    @Override
    public boolean isFinished() {

        if (System.currentTimeMillis() - startScoreTime > MAX_TIME_TO_SCORE_MILLIS) {
            setFinishReason("Score timeout exceeded - aborting");
            return true;
        }

        // ensure that we have a piece and haven't dropped it
        if (!armSubsystem.isGamePieceDetected()) {
            setFinishReason("Can not score, no piece is held.");
            return true;
        }

        // Check position
        if (armSubsystem.isInPosition(scoringPosition)) {

            setFinishReason("ScoreCommand finished");
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
