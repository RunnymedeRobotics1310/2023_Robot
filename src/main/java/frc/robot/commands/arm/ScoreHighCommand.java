package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ScoreHighCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    // fixme: do everything - see table
    // https://docs.google.com/document/d/1JzU-BzCXjGCwosouylmWGN83-x8lv-oPzklcXDqNN2U/edit#

    public ScoreHighCommand(ArmSubsystem armSubsystem) {

        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);

    }

    @Override
    public void initialize() {

        System.out.println("ScoreHighCommand started");

    }

    @Override
    public void execute() {

        // FIXME: do everything
        // pose: score high

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
