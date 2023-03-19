package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.ScoringRow;
import frc.robot.subsystems.ArmSubsystem;

public class ScoreAutoCommand extends BaseArmCommand {

    private final long       MAX_TIME_TO_SCORE_MILLIS = 3000L;

    private final ScoringRow scoringRow;
    private ArmPosition      scoringPosition          = null;
    private GamePiece        gamePiece                = null;

    private enum Step {
        CLEAR_FRAME, LIFT_AND_EXTEND, LOWER_ARM, FINISH
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

        System.out.println("ScoreCommand started.  Scoring row : " + scoringRow);

        printArmState();
        stopArmMotors();

        commandStartTime = System.currentTimeMillis();
        step             = Step.CLEAR_FRAME;
    }

    @Override
    public void execute() {

        switch (step) {

        case CLEAR_FRAME:

            // If the arm is starting inside the frame, then
            // retract before moving the arm.
            if (armSubsystem.getArmLiftAngle() < ArmConstants.CLEAR_FRAME_ARM_ANGLE) {

                if (!moveArmExtendToEncoderCount(0, 1)) {
                    return;
                }

                System.out.println("ScoreAutoCommand: Arm Retracted : switching to lift");
                step = Step.LIFT_AND_EXTEND;
                return;
            }

            return;

        case LIFT_AND_EXTEND:

            // Lift the arm until in position
            if (moveArmLiftToAngle(scoringPosition.angle)) {

                // Then extend
                if (moveArmExtendToEncoderCount(scoringPosition.extension, .7)) {

                    System.out.println("ScoreAutoCommand: Arm lifted and extended : switching to lowering");
                    step              = Step.LOWER_ARM;
                    lowerArmStartTime = System.currentTimeMillis();

                    stopArmMotors();
                    return;
                }
            }

            return;

        case LOWER_ARM:

            // Do not lower the arm if already on the lowest scoring position
            if (scoringRow == ScoringRow.BOTTOM) {
                step = Step.FINISH;
                return;
            }

            // Lower the arm onto the post
            // Lower the arm by 5 degrees or .5 seconds whichever comes first.
            if (moveArmLiftToAngle(scoringPosition.angle - 5)
                || System.currentTimeMillis() - lowerArmStartTime > 500) {

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
            return true;
        }

        // If the command has been running for more than 3 seconds, then end.
        if (System.currentTimeMillis() - commandStartTime > MAX_TIME_TO_SCORE_MILLIS) {
            System.out.println("ScoreAutoCommand: Timed out after 3 seconds");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        stopArmMotors();

        if (interrupted) {
            System.out.print("ScoreCommand interrupted");
        }
        else {
            System.out.print("ScoreCommand ended");
        }
        printArmState();
    }
}
