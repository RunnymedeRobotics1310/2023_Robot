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

        startingPincherPosition = armSubsystem.getPincherEncoder();

        logCommandStart("Starting pincher position " + startingPincherPosition);
    }

    @Override
    public void execute() {

        // Open the pinchers by the release amount.
        openPincher();
    }

    @Override
    public boolean isFinished() {

        if (armSubsystem.isPincherOpen()) {
            setFinishReason("Pinchers fully open");
            return true;
        }

        // if the sensor is not picking up the piece but it is there, open a min amt before giving up
        if (armSubsystem.getPincherEncoder() < startingPincherPosition - 20
            && !armSubsystem.isGamePieceDetected()) {
            setFinishReason("Pinchers opened somewhat and no game piece");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        stopArmMotors();

        logCommandEnd(interrupted);

        // If in teleop, then automatically schedule the robot move to compact state
        if (DriverStation.isTeleopEnabled()) {
            CommandScheduler.getInstance().schedule(new CompactCommand(armSubsystem));
        }
    }
}
