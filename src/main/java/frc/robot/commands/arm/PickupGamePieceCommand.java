package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.subsystems.ArmSubsystem;

public class PickupGamePieceCommand extends BaseArmCommand {

    private GamePiece gamePiece                 = null;

    ArmPosition       targetArmPosition         = null;
    double            targetPincherEncoderCount = 0;

    public PickupGamePieceCommand(GamePiece gamePiece, ArmSubsystem armSubsystem) {

        super(armSubsystem);

        this.gamePiece = gamePiece;
    }

    @Override
    public void initialize() {

        // Resolve the game piece from the operator input.
        System.out.println("StartPickupGamePieceCommand.  GamePiece " + gamePiece);

        printArmState();
        stopArmMotors();

        targetPincherEncoderCount = gamePiece.pincherEncoderCount;

        if (gamePiece == GamePiece.CUBE) {
            targetArmPosition = ArmConstants.DRIVE_WITH_CUBE_POSITION;
        }
        else if (gamePiece == GamePiece.CONE) {
            targetArmPosition = ArmConstants.DRIVE_WITH_CONE_POSITION;
        }
    }

    @Override
    public void execute() {

        // Close on the game piece first
        if (movePincherToEncoderCount(targetPincherEncoderCount)) {

            // If above the target, then retract first
            if (armSubsystem.getArmLiftAngle() > targetArmPosition.angle) {

                if (moveArmExtendToEncoderCount(targetArmPosition.extension, 0.5)) {
                    moveArmLiftToAngle(targetArmPosition.angle);
                }

            }
            else {
                // When below the target, lift first.
                if (moveArmLiftToAngle(targetArmPosition.angle)) {

                    // Retract arm
                    moveArmExtendToEncoderCount(targetArmPosition.extension, 0.5);
                }

            }
        }
    }

    @Override
    public boolean isFinished() {

        // If the arm is at the target position.
        if (armSubsystem.isInPosition(targetArmPosition)) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        stopArmMotors();

        if (interrupted) {
            System.out.print("PickupGamePieceCommand interrupted");
        }
        else {
            System.out.print("PickupGamePieceCommand ended");
        }
        printArmState();
    }
}
