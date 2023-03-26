package frc.robot.commands.arm;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPositionCommand extends BaseArmCommand {

    private final Constants.ArmPosition position;
    public MoveArmToPositionCommand(Constants.ArmPosition position, ArmSubsystem armSubsystem) {
        super(armSubsystem);
        this.position = position;
    }

    @Override
    public void initialize() {
        logCommandStart("Moving arm to position "+position, armSubsystem);
    }

    @Override
    public void execute() {
        // danger - no safety code present
        armSubsystem.moveArmLiftToAngle(position.angle);
        moveArmExtendToEncoderCount(position.extension, .5);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isArmAtLiftAngle(position.angle) && armSubsystem.isAtExtendPosition(position.extension);
    }

    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }
}
