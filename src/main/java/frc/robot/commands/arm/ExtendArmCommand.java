package frc.robot.commands.arm;

import static frc.robot.Constants.ArmConstants.MAX_EXTEND_SPEED;

import frc.robot.subsystems.ArmSubsystem;

public class ExtendArmCommand extends BaseArmCommand {

    final double count;
    public ExtendArmCommand(double count, ArmSubsystem armSubsystem) {
        super(armSubsystem);
        this.count = count;
    }

    @Override
    public void execute() {
        moveArmExtendToEncoderCount(count, MAX_EXTEND_SPEED);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isAtExtendPosition(count);
    }
}
