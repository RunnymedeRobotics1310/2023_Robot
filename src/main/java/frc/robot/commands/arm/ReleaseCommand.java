package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;

public class ReleaseCommand extends BaseArmCommand {

    double startingPincherPosition = 0;

    public ReleaseCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ReleaseCommand started");
        printArmState();

        startingPincherPosition = armSubsystem.getPincherEncoder();
    }

    @Override
    public void execute() {

        // Open the pinchers by the release amount.
        openPincher();
    }

    @Override
    public boolean isFinished() {

        if (armSubsystem.isPincherOpen()) {
            return true;
        }

        return armSubsystem.getPincherEncoder() < startingPincherPosition - 35
            && !armSubsystem.isGamePieceDetected();
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
