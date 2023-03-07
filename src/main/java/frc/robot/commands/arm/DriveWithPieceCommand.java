package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;

public class DriveWithPieceCommand extends BaseArmCommand {

    /**
     * Exposess moveToDriveWithPiece pose in command form so that it can be included in an auto routine. Not expected to be
     * invoked by an operator directly.
     *
     * @param armSubsystem
     */
    public DriveWithPieceCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("DriveWithPieceCommand started");
        stopArmMotors();
        printArmState();
    }

    @Override
    public void execute() {
        moveToDriveWithPiecePose();
    }

    @Override
    public boolean isFinished() {
        return moveToDriveWithPiecePose();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DriveWithPiece Command end. " + (interrupted ? "INTERRUPTED" : "NOT INTERRUPTED"));
    }
}
