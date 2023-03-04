package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.ScoringRow;
import frc.robot.subsystems.ArmSubsystem;

public class ScoreHighCommand extends BaseArmCommand {

    private final ArmSubsystem armSubsystem;

    // fixme: do everything - see table
    // https://docs.google.com/document/d/1JzU-BzCXjGCwosouylmWGN83-x8lv-oPzklcXDqNN2U/edit#

    public ScoreHighCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);

        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);

    }

    @Override
    public void initialize() {

        System.out.println("ScoreHighCommand started");
        printArmState();
        stopArmMotors();

    }

    @Override
    public void execute() {

        GamePiece   gamePiece       = armSubsystem.getHeldGamePiece();
        ArmPosition scoringPosition = ArmConstants.getScoringPosition(gamePiece, ScoringRow.TOP);

        // ensure that we have a piece
        if (gamePiece == GamePiece.NONE) {
            System.out.print("Can not score high, no piece is held.");
        }

        // Move arm to scoring position
        boolean lift = moveArmLiftToAngle(scoringPosition.angle, ArmConstants.MAX_LIFT_SPEED);
        if (lift) {
            boolean extend = moveArmExtendToEncoderCount(scoringPosition.extension, ArmConstants.MAX_EXTEND_SPEED);
            if (extend) {
                System.out.print("ScoreHighCommand finished");
            }
        }
    }

    @Override
    public boolean isFinished() {
        GamePiece   gamePiece       = armSubsystem.getHeldGamePiece();
        ArmPosition scoringPosition = ArmConstants.getScoringPosition(gamePiece, ScoringRow.TOP);

        // Check position
        if (armSubsystem.isArmAtLiftAngle(scoringPosition.angle)
            && armSubsystem.isAtExtendPosition(scoringPosition.extension)) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {

        stopArmMotors();
        if (interrupted) {
            System.out.print("ScoreHighCommand interrupted");
        }
        else {
            System.out.print("ScoreHighCommand ended");
        }

    }
}
