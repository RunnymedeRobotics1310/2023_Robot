package frc.robot.commands.arm;

import static frc.robot.Constants.ArmConstants.ARM_EXTEND_POSITION_TOLERANCE;
import static frc.robot.Constants.ArmConstants.CLEAR_FRAME_ARM_ANGLE;
import static frc.robot.Constants.ArmConstants.GROUND_PICKUP_POSITION;
import static frc.robot.Constants.ArmConstants.MAX_EXTEND_SLOW_ZONE_SPEED;
import static frc.robot.Constants.GameConstants.GamePiece.CONE;
import static frc.robot.Constants.GameConstants.GamePiece.CUBE;
import static frc.robot.Constants.GameConstants.GamePiece.NONE;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.commands.vision.SwitchVisionTargetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionTargetType;

/**
 * Pickup from ground
 * <p>
 * see: https://docs.google.com/document/d/1JzU-BzCXjGCwosouylmWGN83-x8lv-oPzklcXDqNN2U/edit#
 */
public class PickUpGroundCommand extends BaseArmCommand {

    private final VisionSubsystem visionSubsystem;
    private final OperatorInput   operatorInput;

    private GamePiece             pickupTarget;

    private enum State {

        ARM_RETRACTING,
        SAFE_TO_MOVE, // raise arm to safe height
        ARM_MOVING, // extend arm, open pincher and raise arm
        ARM_IN_POSITION, // arm is in the proper position for hoovering
        PIECE_DETECTED, // piece has been found
        PINCHERS_MOVING, // close pinchers
        PIECE_GRABBED, // moving to driving position
        DRIVING_POSITION, // done
        CANCELLED // aborted
    }

    private State state = State.ARM_RETRACTING;

    /**
     * Pick up a piece from the ground in auto.
     *
     * @param gamePiece to pick up
     * @param armSubsystem the arm subsystem
     */
    public PickUpGroundCommand(GamePiece gamePiece, ArmSubsystem armSubsystem) {
        this(gamePiece, null, armSubsystem, null);
    }

    /**
     * Pick up a piece from the ground. This command will remain active until a piece is grasped or
     * it is interrupted or
     * cancelled.
     *
     * @param initialPickupTarget the type of game piece to be picked up - unless later overridden
     * by the operator
     * @param operatorInput optional operator input object which allows the operator to change the
     * selected game piece after the
     * command has started. If not specified, the initialPickupTarget will remain the active pickup
     * target for the duration of the
     * command.
     * @param armSubsystem the arm subsystem
     * @param visionSubsystem the vision subsystem
     */
    public PickUpGroundCommand(GamePiece initialPickupTarget, OperatorInput operatorInput, ArmSubsystem armSubsystem,
        VisionSubsystem visionSubsystem) {

        super(armSubsystem);

        this.visionSubsystem = visionSubsystem;
        this.operatorInput   = operatorInput;
        this.pickupTarget    = initialPickupTarget;

    }

    @Override
    public void initialize() {
        printStatus("initialize");
        // Try to figure out the right state to start
        if (isGroundPickupPose()) {
            state = State.ARM_IN_POSITION;
            stopArmMotors();
        }
        else if (armSubsystem.isArmRetracted()) {
            state = State.SAFE_TO_MOVE;
            stopArmMotors();
        }
        else if (armSubsystem.isArmInsideFrame()) {
            state = State.ARM_RETRACTING;
        }
        else {
            state = State.ARM_MOVING;
        }

        setVisionTarget(pickupTarget);
        printStatus("end of initialize");

    }

    @Override
    public void execute() {

        if (operatorInput != null) {
            // check for operator override. If both selected, cube wins. If none selected, command
            // is cancelled.
            if (operatorInput.isPickUpCone()) {
                if (pickupTarget == CUBE) {
                    System.out.println("Changing pickup target from CUBE to CONE based on operator input");
                    pickupTarget = CONE;
                    setVisionTarget(pickupTarget);
                }
            }
            else if (operatorInput.isPickUpCube()) {
                if (pickupTarget == CONE) {
                    System.out.println("Changing pickup target from CONE to CUBE based on operator input");
                    pickupTarget = CUBE;
                    setVisionTarget(pickupTarget);
                }
            }
            else {
                pickupTarget = NONE;
                printStatus("Changing state");
                state = State.CANCELLED;
            }
        }


        switch (state) {

        case ARM_RETRACTING: {
            boolean compactDone = moveArmExtendToEncoderCount(0, MAX_EXTEND_SLOW_ZONE_SPEED);
            if (compactDone) {
                printStatus("Changing state");
                state = State.SAFE_TO_MOVE;
            }
            break;
        }

        case SAFE_TO_MOVE: {
            // sets arm position to ground pickup
            boolean liftDone = moveArmLiftToAngle(CLEAR_FRAME_ARM_ANGLE);
            if (liftDone) {
                printStatus("Changing state");
                state = State.ARM_MOVING;
            }
            break;
        }

        case ARM_MOVING: {
            // Always open the pincher
            boolean pinchDone = openPincher();
            // The arm is already lifted to a safe angle,
            // so set the extension before lowering the arm.
            if (moveArmExtendToEncoderCount(GROUND_PICKUP_POSITION.extension, .5)) {
                boolean liftDone = moveArmLiftToAngle(
                    Double.max(CLEAR_FRAME_ARM_ANGLE, GROUND_PICKUP_POSITION.angle));
                if (liftDone && pinchDone) {
                    printStatus("Changing state");
                    state = State.ARM_IN_POSITION;
                }
            }
            break;
        }

        case ARM_IN_POSITION: {
            if (armSubsystem.isGamePieceDetected()) {
                printStatus("Changing state");
                state = State.PIECE_DETECTED;
            }
            break;
        }

        case PIECE_DETECTED:
        case PINCHERS_MOVING: {
            boolean pinchDone = movePincherToEncoderCount(pickupTarget.pincherEncoderCount);
            printStatus("Changing state");
            state = pinchDone ? State.PIECE_GRABBED : State.PINCHERS_MOVING;
            break;
        }

        case PIECE_GRABBED:
            boolean armInPositionDone = moveToDriveWithPiecePose();
            printStatus("Changing state");
            state = armInPositionDone ? State.DRIVING_POSITION : State.PIECE_GRABBED;
            break;

        case CANCELLED: {
            // note when this occurs the robot remains in this state.
            System.out.println("No pickup target specified by operator. Cancelling command.");
            break;
        }

        default:
            printStatus("unknown state encountered");
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.CANCELLED
            || (state == State.DRIVING_POSITION && armSubsystem.getHeldGamePiece() != NONE);
    }

    @Override
    public void end(boolean interrupted) {

        printStatus("end. Interrupted? " + interrupted);
        stopArmMotors();

        // If in teleop and the game piece is dropped or the command is cancelled, go to compact pose
        if (armSubsystem.getHeldGamePiece() == GamePiece.NONE
            && DriverStation.isTeleopEnabled()) {

            CommandScheduler.getInstance().schedule(new CompactCommand(armSubsystem));
        }
    }

    private boolean isGroundPickupPose() {
        if (!armSubsystem.isArmAtLiftAngle(GROUND_PICKUP_POSITION.angle))
            return false;
        double ext = armSubsystem.getArmExtendEncoder();
        if (Math.abs(ext - GROUND_PICKUP_POSITION.extension) > ARM_EXTEND_POSITION_TOLERANCE)
            return false;
        if (!armSubsystem.isPincherOpen())
            return false;
        return true;
    }

    private void printStatus(String msg) {
        System.out.println("PickupGroundCommand: " + msg);
        System.out.println("PickupGroundCommand: Target: " + pickupTarget);
        System.out.println("PickupGroundCommand: State: " + state);
        printArmState();
    }

    private void setVisionTarget(GamePiece gamePiece) {

        // Do not change the vision target unless the vision subsystem is passed in.
        if (visionSubsystem == null) {
            return;
        }

        VisionTargetType visionTargetType = null;

        switch (gamePiece) {

        case CONE:
            visionTargetType = VisionTargetType.CONE;
            break;

        case CUBE:
            visionTargetType = VisionTargetType.CUBE;
            break;

        default:
            break;
        }

        // Schedule a command to set the vision target
        if (visionTargetType != null
            && visionSubsystem.getCurrentVisionTargetType() != visionTargetType) {

            CommandScheduler.getInstance()
                .schedule(new SwitchVisionTargetCommand(visionTargetType, visionSubsystem));
        }

    }
}
