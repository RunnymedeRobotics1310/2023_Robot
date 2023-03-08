package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ReleaseCommand extends BaseArmCommand {

    private double targetPincherPosition = 0;

    public ReleaseCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ReleaseCommand started");
        printArmState();

        // Open the pincher by the amount required to release the game piece.
        targetPincherPosition = Math.max(0, armSubsystem.getPincherEncoder() - ArmConstants.PINCHER_GAME_PIECE_RELEASE_DISTANCE);
    }

    @Override
    public void execute() {

        // Open the pinchers by the release amount.
        movePincherToEncoderCount(targetPincherPosition);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isAtPincherPosition(targetPincherPosition);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ReleaseCommand end.  " + (interrupted ? "Interrupted." : "Not interrupted"));
        printArmState();
        stopArmMotors();

        // If in teleop, then automatically schedule the robot move to compact state
        if (DriverStation.isTeleopEnabled()) {
            CommandScheduler.getInstance().schedule(new CompactCommand(armSubsystem));
        }
    }
}
