package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.ScoringRow;
import frc.robot.subsystems.ArmSubsystem;

public class ScoreCommand extends BaseArmCommand {

    private final ArmSubsystem armSubsystem;
    private final ScoringRow   scoringRow;
    private ArmPosition        scoringPosition = null;
    private GamePiece          gamePiece       = null;

    public ScoreCommand(ArmSubsystem armSubsystem, ScoringRow scoringRow) {
        super(armSubsystem);

        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
        this.scoringRow = scoringRow;
    }

    @Override
    public void initialize() {

        gamePiece       = armSubsystem.getHeldGamePiece();
        scoringPosition = ArmConstants.getScoringPosition(gamePiece, scoringRow);

        System.out.println("ScoreCommand started.  Scoring row : " + scoringRow);

        printArmState();
        stopArmMotors();
    }

    @Override
    public void execute() {

        // Move arm to scoring position
        boolean lift = moveArmLiftToAngle(scoringPosition.angle, ArmConstants.MAX_LIFT_SPEED);
        if (lift) {
            moveArmExtendToEncoderCount(scoringPosition.extension, ArmConstants.MAX_EXTEND_SPEED);
        }
    }

    @Override
    public boolean isFinished() {

        // ensure that we have a piece and haven't dropped it
        if (!armSubsystem.isGamePieceDetected()) {
            System.out.print("Can not score, no piece is held.");
            return true;
        }

        // Check position
        if (armSubsystem.isArmAtLiftAngle(scoringPosition.angle)
            && armSubsystem.isAtExtendPosition(scoringPosition.extension)) {

            System.out.print("ScoreCommand finished");
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
