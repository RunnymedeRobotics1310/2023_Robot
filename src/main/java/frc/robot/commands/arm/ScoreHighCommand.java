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

        // FIXME: What happens if we drop the piece while trying to position the arm?
        GamePiece   gamePiece       = armSubsystem.getHeldGamePiece();

        // FIXME: Do we need to call this every time or should it go into the initialize routine?
        ArmPosition scoringPosition = ArmConstants.getScoringPosition(gamePiece, ScoringRow.TOP);

        // ensure that we have a piece
        // FIXME: If this is part of the isFinished() routine, it is not required here.
        // How should this command finish?
        // Maybe it finishes differently in auto than in teleop?
        if (gamePiece == GamePiece.NONE) {
            System.out.print("Can not score high, no piece is held.");
        }

        // Move arm to scoring position
        boolean lift = moveArmLiftToAngle(scoringPosition.angle, ArmConstants.MAX_LIFT_SPEED);
        if (lift) {
            boolean extend = moveArmExtendToEncoderCount(scoringPosition.extension, ArmConstants.MAX_EXTEND_SPEED);
            if (extend) {
                // FIXME: Do not print this message here, it should be in the isFinished routine.
                System.out.print("ScoreHighCommand finished");
            }
        }

        // FIXME: Is this command responsible for the trimming? RECOMMENDED.
        // Need to pass in the OperatorInput into this command in order to trim the height
        // and extension of the arm.
    }

    @Override
    public boolean isFinished() {

        // TELEOP:
        // if (DriverStation.isTeleopEnabled())..
        // FIXME: If this command does the trimming (recommended), then do not end this command
        // It will be ended when it is interrupted by the ReleaseCommand.
        // return false;

        // AUTO:
        // IF there is not a game piece, then the command should finish.
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
