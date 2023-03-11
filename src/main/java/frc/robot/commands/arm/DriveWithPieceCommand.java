package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.GameConstants.GamePiece;
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
        // Nothing to do here (the execute loop happens in isFinished
    }

    @Override
    public boolean isFinished() {
        return moveToDriveWithPiecePose();
    }

    @Override
    public void end(boolean interrupted) {

        System.out.println("DriveWithPiece Command end. " + (interrupted ? "INTERRUPTED" : "NOT INTERRUPTED"));
        printArmState();

        // If there is no game piece, and the system is in teleop, then
        // go back to compact pose.
        if (armSubsystem.getHeldGamePiece() == GamePiece.NONE
            && DriverStation.isTeleopEnabled()) {

            CommandScheduler.getInstance().schedule(new CompactCommand(armSubsystem));
        }

    }
}
