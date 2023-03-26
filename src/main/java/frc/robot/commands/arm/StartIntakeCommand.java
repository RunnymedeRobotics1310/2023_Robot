package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.VisionConstants.VisionTarget;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.commands.vision.SetVisionTargetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.Constants.ArmConstants.GROUND_PICKUP_AUTO_POSITION;
import static frc.robot.Constants.ArmConstants.GROUND_PICKUP_POSITION;

public class StartIntakeCommand extends BaseArmCommand {

    private final OperatorInput   operatorInput;
    private final VisionSubsystem visionSubsystem;

    private final Constants.ArmPosition groundPickupPosition;
    private GamePiece             gamePiece    = null;
    private VisionTarget          visionTarget = null;

    public StartIntakeCommand(GamePiece gamePiece, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem) {

        super(armSubsystem);

        this.groundPickupPosition = GROUND_PICKUP_AUTO_POSITION;
        this.gamePiece       = gamePiece;
        this.operatorInput   = null;
        this.visionSubsystem = visionSubsystem;
    }

    public StartIntakeCommand(OperatorInput operatorInput, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem) {

        super(armSubsystem);

        this.groundPickupPosition = GROUND_PICKUP_POSITION;

        this.operatorInput   = operatorInput;
        this.visionSubsystem = visionSubsystem;

        this.gamePiece       = GamePiece.NONE;
    }

    @Override
    public void initialize() {

        // Resolve the game piece from the operator input.
        if (operatorInput != null) {

            if (operatorInput.isPickUpCube()) {
                gamePiece    = GamePiece.CUBE;
                visionTarget = VisionTarget.CUBE_GROUND;
            }
            else {
                if (operatorInput.isPickUpCone()) {
                    gamePiece    = GamePiece.CONE;
                    visionTarget = VisionTarget.CONE_GROUND;
                }
            }

            if (gamePiece != GamePiece.NONE) {
                // Set the vision target in the vision subsystem.
                CommandScheduler.getInstance().schedule(new SetVisionTargetCommand(visionTarget, visionSubsystem));
            }
        }

        stopArmMotors();

        logCommandStart("GamePiece " + gamePiece);
    }

    @Override
    public void execute() {

        // Resolve the game piece from the operator input.
        if (operatorInput != null) {

            if (operatorInput.isPickUpCube() && gamePiece != GamePiece.CUBE) {

                // Adjust the vision target
                gamePiece    = GamePiece.CUBE;
                visionTarget = VisionTarget.CUBE_GROUND;

                CommandScheduler.getInstance().schedule(new SetVisionTargetCommand(visionTarget, visionSubsystem));
            }
            else {
                if (operatorInput.isPickUpCone() && gamePiece != GamePiece.CONE) {

                    // Adjust the vision target
                    gamePiece    = GamePiece.CONE;
                    visionTarget = VisionTarget.CONE_GROUND;

                    CommandScheduler.getInstance().schedule(new SetVisionTargetCommand(visionTarget, visionSubsystem));
                }
            }
        }

        double armAngle          = armSubsystem.getArmLiftAngle();
        double armExtendPosition = armSubsystem.getArmExtendEncoder();

        /*
         * Special logic to make sure the arm comes up over the frame when extending.
         */
        if (armAngle < (ArmConstants.CLEAR_FRAME_ARM_ANGLE - ArmConstants.ARM_LIFT_ANGLE_TOLERANCE_DEGREES)
            && armExtendPosition < ArmConstants.MAX_ARM_EXTEND_INSIDE_FRAME) {

            // Retract the arm before lifting.
            if (!retractArm()) {
                return; // Wait for the retraction before lifting the arm
            }

            moveArmLiftToAngle(ArmConstants.CLEAR_FRAME_ARM_ANGLE + ArmConstants.ARM_LIFT_ANGLE_TOLERANCE_DEGREES + 2);
            return;
        }

        /*
         * Move the extension before lowering the arm below the frame.
         */

        // Always open the pincher, there is no point in waiting
        openPincher();

        if (moveArmExtendToEncoderCount(groundPickupPosition.extension, ArmConstants.MAX_EXTEND_SPEED)) {

            moveArmLiftToAngle(groundPickupPosition.angle);
        }
    }

    @Override
    public boolean isFinished() {

        // If the user is no longer asking for pickup
        if (operatorInput != null) {
            if (!operatorInput.isPickUpCone() && !operatorInput.isPickUpCube()) {
                setFinishReason("Driver cancelled pickup");
                return true;
            }
        }

        // If at the target position, and a game piece is detected.
        if (armSubsystem.isInPosition(groundPickupPosition)) {

            // If a game piece is detected
            if (DriverStation.isTeleopEnabled()) {
                if (armSubsystem.isGamePieceDetected()) {
                    setFinishReason("Game piece detected in pickup in teleop.");
                    return true;
                }
            } else {
                setFinishReason("Arm is in position in auto.");
                return true;

            }
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        stopArmMotors();

        logCommandEnd(interrupted);

        // In Teleop, pick the next command
        if (DriverStation.isTeleopEnabled()) {

            if (armSubsystem.isGamePieceDetected()) {
                CommandScheduler.getInstance().schedule(new PickupGamePieceCommand(gamePiece, operatorInput, armSubsystem));
            }
            else {
                CommandScheduler.getInstance().schedule(new CompactCommand(armSubsystem));
            }
        }
    }
}
