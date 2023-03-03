package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.subsystems.ArmSubsystem;

public class ScoreHighCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    // fixme: do everything - see table
    // https://docs.google.com/document/d/1JzU-BzCXjGCwosouylmWGN83-x8lv-oPzklcXDqNN2U/edit#

    public ScoreHighCommand(ArmSubsystem armSubsystem) {

        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);

    }

    @Override
    public void initialize() {

        System.out.println("ScoreHighCommand started");

        armSubsystem.setArmLiftSpeed(0);
        armSubsystem.setArmExtendSpeed(0);
        armSubsystem.setPincherSpeed(0);

    }

    @Override
    public void execute() {
        // Q is working on this

        // ensure that we have a piece
        GamePiece gamePiece = armSubsystem.getHeldGamePiece();

        if (gamePiece == GamePiece.NONE) {
            System.out.print("Can not score high, no piece is held.");
            // FIXME: This is an infinite loop?
            return;
        }

        // FIXME:
        // Quentin: There are some new constants for the arm, and a helper method to get the scoring
        // position for all of the scoring spots
        // The scoring position has an angle and extension. The isAtLiftAngle should be in the
        // subsystem which can tell you if the current arm position equals the passed in position.
        // FIXME: We should do this for the extension as well.

        ArmPosition scoringPosition = ArmConstants.getScoringPosition(gamePiece, GameConstants.ScoringRow.TOP);

        if (armSubsystem.isArmAtLiftAngle(scoringPosition.angle)) {
            armSubsystem.setArmLiftSpeed(0);
        }
        else if (armSubsystem.getArmLiftAngle() < scoringPosition.angle) {
            armSubsystem.setArmLiftSpeed(.25);
        }
        else {
            armSubsystem.setArmLiftSpeed(-.25);
        }

        // Cube arm extend
        // FIXME: Quentin, please add a method to the arm subsystem
        // .isAtExtendPosition(double position)
        // and you can follow the lift angle pattern above for the extension.

        // TODO: determine if there are timing considerations?
        // Does the arm have to be lifted _before_ the extension can happen?
    }

    @Override
    public boolean isFinished() {
        // FIXME: do everything
        // finish

        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // FIXME: do everything
        if (interrupted) {
            System.out.print("ScoreHighCommand interrupted");
        }
        else {
            System.out.print("ScoreHighCommand ended");
        }

    }
}
