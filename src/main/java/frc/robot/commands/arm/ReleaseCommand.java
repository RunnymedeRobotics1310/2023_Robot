package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;

public class ReleaseCommand extends BaseArmCommand {

    private final ArmSubsystem armSubsystem;

    public ReleaseCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ReleaseCommand started");
        printArmState();
    }

    @Override
    public void execute() {
        if (armSubsystem.isGamePieceDetected()) {
            movePincherToEncoderCount(0);
        }
        else {
            // as soon as the game piece is NO LONGER detected the pincher motor
            // reverses direction to go into compact pose
            moveToCompactPose();
        }
    }

    @Override
    public boolean isFinished() {
        return !armSubsystem.isGamePieceDetected() && isCompactPose();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ReleaseCommand end.  " + (interrupted ? "Interrupted." : "Not interrupted"));
        printArmState();
        stopArmMotors();
    }
}
