package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToAngleCommand extends BaseArmCommand {

    private final double angle;
    public MoveArmToAngleCommand(double angle, ArmSubsystem armSubsystem) {
        super(armSubsystem);
        this.angle = angle;
    }

    @Override
    public void execute() {
        armSubsystem.moveArmLiftToAngle(angle);
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isArmAtLiftAngle(angle);
    }
}
