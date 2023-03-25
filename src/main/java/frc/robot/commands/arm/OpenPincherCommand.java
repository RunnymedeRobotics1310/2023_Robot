package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;

public class OpenPincherCommand extends BaseArmCommand {
    public OpenPincherCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);
    }

    @Override
    public void execute() {
        openPincher();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isPincherOpen();
    }
}
