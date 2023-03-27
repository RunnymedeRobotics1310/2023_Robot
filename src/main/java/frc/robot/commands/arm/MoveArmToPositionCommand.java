package frc.robot.commands.arm;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPositionCommand extends BaseArmCommand {

    private final Constants.ArmPosition position;

    public MoveArmToPositionCommand(Constants.ArmPosition position, ArmSubsystem armSubsystem) {
        super(armSubsystem);
        this.position = position;
    }

    @Override
    public void initialize() {
        logCommandStart("Moving arm to position " + position);
    }

    @Override
    public void execute() {

        // ensure that arm extend does not slam into ground
        if (armSubsystem.getArmExtendEncoder() > position.extension) {

            if (moveArmExtendToEncoderCount(position.extension, ArmConstants.MAX_EXTEND_SPEED)) {
                armSubsystem.moveArmLiftToAngle(position.angle);
            }
        }
        else {
            armSubsystem.moveArmLiftToAngle(position.angle);
            moveArmExtendToEncoderCount(position.extension, ArmConstants.MAX_EXTEND_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isArmAtLiftAngle(position.angle) && armSubsystem.isAtExtendPosition(position.extension);
    }
}
