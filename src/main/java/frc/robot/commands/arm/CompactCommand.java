package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;

public class CompactCommand extends BaseArmCommand {

    public CompactCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);
        addRequirements(armSubsystem);
    }

    private void printStatus(String msg) {
        System.out.println("Compact Command: " + msg);
        printArmState();
    }

    @Override
    public void initialize() {
        printStatus("initialize");
        if (isCompactPose()) {
            stopArmMotors();
        }
    }

    @Override
    public void execute() {

        if (isCompactPose()) {
            return;
        }
        else {
            moveToCompactPose();
        }
    }

    @Override
    public boolean isFinished() {
        return isCompactPose();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            printStatus("End. Interrupted");
        }
        else {
            printStatus("End. Not interrupted");
        }
        stopArmMotors();
    }
}
