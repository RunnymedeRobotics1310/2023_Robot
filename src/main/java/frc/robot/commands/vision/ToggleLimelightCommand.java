package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class ToggleLimelightCommand extends CommandBase {

    private final VisionSubsystem visionSubsystem;

    // fixme: do everything - see table
    // https://docs.google.com/document/d/1JzU-BzCXjGCwosouylmWGN83-x8lv-oPzklcXDqNN2U/edit#

    public ToggleLimelightCommand(VisionSubsystem visionSubsystem) {

        this.visionSubsystem = visionSubsystem;

        addRequirements(visionSubsystem);

    }

    @Override
    public void initialize() {

        System.out.println("ToggleLimelightCommand started");

    }

    @Override
    public void execute() {

        // FIXME: do everything
        // toggle limelight between looking up and looking down

        ;

    }

    @Override
    public boolean isFinished() {
        // FIXME: do everything
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // FIXME: do everything
    }
}
