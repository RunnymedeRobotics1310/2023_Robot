package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

    }

    @Override
    public void execute() {

        // Open the pinchers by the release amount.
        openPincher();
    }

    @Override
    public boolean isFinished() {
        return !armSubsystem.isGamePieceDetected();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ReleaseCommand end.  " + (interrupted ? "Interrupted." : "Not interrupted"));
        printArmState();
        stopArmMotors();

        // If in teleop, then automatically schedule the robot move to compact state
        if (DriverStation.isTeleopEnabled()) {
            CommandScheduler.getInstance().schedule(new CompactCommand2(armSubsystem));
        }
    }
}
