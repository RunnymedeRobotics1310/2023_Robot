package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.VisionConstants.VisionTarget;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class StartIntakeCommand extends BaseArmCommand {

    private final OperatorInput   operatorInput;
    private final VisionSubsystem visionSubsystem;

    private GamePiece             gamePiece = null;

    public StartIntakeCommand(GamePiece gamePiece, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem) {

        super(armSubsystem);

        this.gamePiece       = gamePiece;
        this.operatorInput   = null;
        this.visionSubsystem = visionSubsystem;
    }

    public StartIntakeCommand(OperatorInput operatorInput, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem) {

        super(armSubsystem);

        this.operatorInput   = operatorInput;
        this.visionSubsystem = visionSubsystem;

        this.gamePiece       = GamePiece.NONE;
    }

    @Override
    public void initialize() {

        // Resolve the game piece from the operator input.
        if (operatorInput != null) {

            if (operatorInput.isPickUpCube()) {
                gamePiece = GamePiece.CUBE;
            }
            else {
                if (operatorInput.isPickUpCone()) {
                    gamePiece = GamePiece.CONE;
                }
            }
        }

        System.out.println("StartIntakeCommand started.  GamePiece " + gamePiece);

        // Start the vision based on the game piece
        if (gamePiece == GamePiece.CUBE) {

            visionSubsystem.setVisionTarget(VisionTarget.CUBE_GROUND);

        }
        else if (gamePiece == GamePiece.CONE) {

            visionSubsystem.setVisionTarget(VisionTarget.CONE_GROUND);

        }

        printArmState();
        stopArmMotors();
    }

    @Override
    public void execute() {

        // Resolve the game piece from the operator input.
        if (operatorInput != null) {

            if (operatorInput.isPickUpCube()) { // && gamePiece != GamePiece.CUBE) {

                // System.out.println("StartIntakeCommand: Game Piece switched to CUBE.");
                gamePiece = GamePiece.CUBE;
                visionSubsystem.setVisionTarget(VisionTarget.CUBE_GROUND);
            }
            else {
                if (operatorInput.isPickUpCone()) {

                    // System.out.println("StartIntakeCommand: Game Piece switched to CONE.");
                    gamePiece = GamePiece.CONE;
                    visionSubsystem.setVisionTarget(VisionTarget.CONE_GROUND);
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

        if (moveArmExtendToEncoderCount(ArmConstants.GROUND_PICKUP_POSITION.extension, ArmConstants.MAX_EXTEND_SPEED)) {

            moveArmLiftToAngle(ArmConstants.GROUND_PICKUP_POSITION.angle);
        }
    }

    @Override
    public boolean isFinished() {

        // If the user is no longer asking for pickup
        if (operatorInput != null) {
            if (!operatorInput.isPickUpCone() && !operatorInput.isPickUpCube()) {
                return true;
            }
        }

        // If at the target position, and a game piece is detected.
        if (armSubsystem.isInPosition(ArmConstants.GROUND_PICKUP_POSITION)) {

            // If a game piece is detected
            if (armSubsystem.isGamePieceDetected()) {
                return true;
            }
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
