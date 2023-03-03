package frc.robot.commands.arm;

import
    edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.ArmConstants.*;

public class PickUpGroundCommand extends CommandBase {

    private final ArmSubsystem  armSubsystem;
    private final OperatorInput operatorInput;

    // https://docs.google.com/document/d/1JzU-BzCXjGCwosouylmWGN83-x8lv-oPzklcXDqNN2U/edit#

    /**
     * Pick up a piece from the ground. This command will remain active until a piece is grasped or it is interrupted or
     * cancelled.
     *
     * @param operatorInput the controllers
     * @param armSubsystem the arm subsystem
     */
    public PickUpGroundCommand(OperatorInput operatorInput, ArmSubsystem armSubsystem) {

        this.armSubsystem  = armSubsystem;
        this.operatorInput = operatorInput;

        addRequirements(armSubsystem);

    }


    private enum State {
        ARM_RETRACTING, COMPAT_POSE, ARM_MOVING, ARM_IN_POSITION, PIECE_DETECTED, PINCHERS_MOVING, PIECE_GRABBED
    }

    private State state = State.ARM_RETRACTING;

    private void printStatus(String msg) {
        System.out.println("PickupGroundCommand: " + msg);
        System.out.println("PickupGroundCommand: Arm height: " + armSubsystem.getArmLiftEncoder());
        System.out.println("PickupGroundCommand: Arm extent: " + armSubsystem.getArmExtendEncoder());
        System.out.println("PickupGroundCommand: Pincher: " + armSubsystem.getPincherEncoder());
        System.out.println("PickupGroundCommand: State: " + state);
    }

    @Override
    public void initialize() {
        printStatus("initialize");
        // dump arm data
        if (armSubsystem.isArmRetracted()) {
            state = State.COMPAT_POSE;
            armSubsystem.setArmExtendSpeed(0);
        }
        else {
            armSubsystem.setArmExtendSpeed(-0.1);
            state = State.ARM_RETRACTING;
        }
    }

    private final double CLEAR_FRAME_LIFT_ENCODER_LOCATION = 3; // todo: fixme - get real value

    @Override
    public void execute() {
        printStatus("execute");

        switch (state) {
        case ARM_RETRACTING: {
            if (armSubsystem.isArmRetracted()) {
                state = State.COMPAT_POSE;
                armSubsystem.setArmExtendSpeed(0);
            }
            else {
                armSubsystem.setArmExtendSpeed(-0.1);
                state = State.ARM_RETRACTING;
            }
        }
        case COMPAT_POSE: {
            boolean moving = false;
            // sets arm position to ground pickup
            if ((armSubsystem.getArmLiftEncoder() - GROUND_PICKUP_HEIGHT) > ARM_LIFT_MOTOR_TOLERANCE) {
                armSubsystem.setArmLiftSpeed(.25);
                moving = true;
            }
            else if ((armSubsystem.getArmLiftEncoder() - GROUND_PICKUP_HEIGHT) < ARM_LIFT_MOTOR_TOLERANCE) {
                armSubsystem.setArmLiftSpeed(-1);
                moving = true;
            }
            else {
                armSubsystem.setArmLiftSpeed(0);
            }

            // don't extend arm or open pinchers if we haven't extended raised the arm above the frame

            if (armSubsystem.getArmLiftEncoder() >= CLEAR_FRAME_LIFT_ENCODER_LOCATION) {

                // set extension
                if ((armSubsystem.getArmExtendEncoder() - GROUND_PICKUP_EXTEND) > ARM_EXTEND_MOTOR_TOLERANCE) {
                    armSubsystem.setArmExtendSpeed(.25);
                    moving = true;
                }
                else if ((armSubsystem.getArmLiftEncoder() - GROUND_PICKUP_EXTEND) < ARM_EXTEND_MOTOR_TOLERANCE) {
                    armSubsystem.setArmExtendSpeed(-.25);
                    moving = true;
                }
                else {
                    armSubsystem.setArmExtendSpeed(0);
                }

                // set pinchers
                if (!armSubsystem.isPincherOpen()) {
                    armSubsystem.setPincherSpeed(-.25);
                    moving = true;
                }
                else {
                    armSubsystem.setPincherSpeed(0);
                }
            }

            state = moving ? State.ARM_MOVING : State.ARM_IN_POSITION;
        }
        case ARM_IN_POSITION: {
            if (armSubsystem.isGamePieceDetected()) {
                state = State.PIECE_DETECTED;
            }
        }
        case PIECE_DETECTED: {
            if (operatorInput.isPickUpCube()
                && (armSubsystem.getPincherEncoder() - GamePiece.CUBE.pincherEncoderCount) < PINCHER_MOTOR_TOLERANCE) {
                armSubsystem.setPincherSpeed(.25);
                state = State.PINCHERS_MOVING;
            }
            else if (operatorInput.isPickUpCone()
                && (armSubsystem.getPincherEncoder() - GamePiece.CONE.pincherEncoderCount) < PINCHER_MOTOR_TOLERANCE) {
                armSubsystem.setPincherSpeed(.25);
                state = State.PINCHERS_MOVING;
            }
            else {
                armSubsystem.setPincherSpeed(0);
                state = State.PIECE_GRABBED;
            }

        }
        }

    }

    @Override
    public boolean isFinished() {
        printStatus("finished");
        return state == State.PIECE_GRABBED;
    }

    @Override
    public void end(boolean interrupted) {
        printStatus("interrupted: " + interrupted);
        armSubsystem.setArmLiftSpeed(0);
        armSubsystem.setArmExtendSpeed(0);
        armSubsystem.setPincherSpeed(0);
    }
}
