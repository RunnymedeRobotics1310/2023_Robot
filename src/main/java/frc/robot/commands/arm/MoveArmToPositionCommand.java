package frc.robot.commands.arm;

import static frc.robot.Constants.ArmConstants.CLEAR_FRAME_ARM_ANGLE;
import static frc.robot.Constants.ArmConstants.MAX_ARM_EXTEND_INSIDE_FRAME;
import static frc.robot.Constants.ArmConstants.SAFELY_CLEAR_FRAME_POSITION;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;


public class MoveArmToPositionCommand extends BaseArmCommand {

    private static final long COMMAND_TIMEOUT_MILLISECONDS = 3000L;

    private enum Step {RETRACT, LIFT_TO_CLEAR_FRAME, LIFT_AND_EXTEND, LOWER_ARM, FINISH}

    private Constants.ArmPosition position;
    private       Step                  step             = Step.FINISH;
    private       long                  commandStartTime = 0L;

    public MoveArmToPositionCommand(Constants.ArmPosition position, ArmSubsystem armSubsystem) {
        super(armSubsystem);
        this.position = position;
    }

    @Override
    public void initialize() {

        stopArmMotors();

        commandStartTime = System.currentTimeMillis();
        double extent = armSubsystem.getArmExtendEncoder();
        double angle = armSubsystem.getArmLiftAngle();

        if (angle >= CLEAR_FRAME_ARM_ANGLE) {

            step = Step.LIFT_AND_EXTEND;

        }
        else {

            if (extent > MAX_ARM_EXTEND_INSIDE_FRAME) {

                step = Step.RETRACT;

            }
            else {

                step = Step.LIFT_TO_CLEAR_FRAME;

            }
        }

        // prevent requesting an unsafe position
        if (position.angle < CLEAR_FRAME_ARM_ANGLE && position.extension > MAX_ARM_EXTEND_INSIDE_FRAME) {

            logCommandStart("Moving arm to position - OVERRIDE: "+position+" is unsafe. Moving to safe position: "+SAFELY_CLEAR_FRAME_POSITION+", Starting Step " + step+"");
            position = SAFELY_CLEAR_FRAME_POSITION;

        } else {

            logCommandStart("Moving arm to position : "+position+ ", Starting Step " + step);

        }


    }

    @Override
    public void execute() {

        switch (step) {

        case RETRACT:

            if (moveArmExtendToEncoderCount(0, ArmConstants.MAX_EXTEND_SPEED)) {

                logStateTransition("RETRACT -> LIFT_TO_CLEAR_FRAME");
                step = Step.LIFT_TO_CLEAR_FRAME;

            }

            return;

        case LIFT_TO_CLEAR_FRAME:

            if (moveArmLiftToAngle(CLEAR_FRAME_ARM_ANGLE)) {

                step = Step.LIFT_AND_EXTEND;
                logStateTransition("LIFT_TO_CLEAR_FRAME -> LIFT_AND_EXTEND");

            }

            return;

        case LIFT_AND_EXTEND:

            // to get here, we are already at or above the clear frame angle
            if (moveArmLiftToAngle(position.angle)) {

                // Then extend
                if (moveArmExtendToEncoderCount(position.extension, .7)) {

                    step              = Step.FINISH;

                    logStateTransition("LIFT_AND_EXTEND -> FINISH");

                    stopArmMotors();
                    return;

                }
            }

            return;

        case FINISH:
        default:
        }

    }

    @Override
    public boolean isFinished() {

        if (step == Step.FINISH) {
            setFinishReason("Arm in position");
            return true;
        }

        // If the command has been running for too long, then end.
        if (System.currentTimeMillis() - commandStartTime > COMMAND_TIMEOUT_MILLISECONDS) {
            setFinishReason("Timed out after "+COMMAND_TIMEOUT_MILLISECONDS+"ms");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        stopArmMotors();

        logCommandEnd(interrupted);
    }
}
