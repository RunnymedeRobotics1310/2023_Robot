package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;

public class PickupGamePieceCommand extends BaseArmCommand {

    private GamePiece gamePiece                 = null;

    ArmPosition       targetArmPosition         = null;
    double            targetPincherEncoderCount = 0;
    OperatorInput     operatorInput             = null;

    public PickupGamePieceCommand(GamePiece gamePiece, OperatorInput operatorInput, ArmSubsystem armSubsystem) {

        super(armSubsystem);

        this.gamePiece     = gamePiece;
        this.operatorInput = operatorInput;
    }

    @Override
    public void initialize() {

        stopArmMotors();

        targetPincherEncoderCount = gamePiece.pincherEncoderCount;

        if (gamePiece == GamePiece.CUBE) {
            targetArmPosition = ArmConstants.DRIVE_WITH_CUBE_POSITION;
        }
        else if (gamePiece == GamePiece.CONE) {
            targetArmPosition = ArmConstants.DRIVE_WITH_CONE_POSITION;
        }

        StringBuilder commandParms = new StringBuilder();
        commandParms.append("GamePiece ").append(gamePiece)
            .append(", Pincher target ").append(targetPincherEncoderCount)
            .append(", Arm target ").append(targetArmPosition);

        logCommandStart(commandParms.toString());
    }

    @Override
    public void execute() {

        // Close on the game piece first
        if (movePincherToEncoderCount(targetPincherEncoderCount)) {

            // Notify the driver that they have a game piece
            if (operatorInput != null) {
                operatorInput.startVibrate();
            }

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
            setFinishReason("arm in position");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        stopArmMotors();

        if (operatorInput != null) {
            operatorInput.stopVibrate();
        }

        logCommandEnd(interrupted);
    }
}
