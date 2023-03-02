package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class DriveWithPieceCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    // fixme: do everything - see table
    // https://docs.google.com/document/d/1JzU-BzCXjGCwosouylmWGN83-x8lv-oPzklcXDqNN2U/edit#

    public DriveWithPieceCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("DriveWithPieceCommand started");
        // fixme
        // stop arms
        // print current position etc.
    }

    @Override
    public void execute() {
        // FIXME: do everything
        // if not holding a piece abort
        if (armSubsystem.isGamePieceDetected()) {
            return;
        }
        // if holding a cone enter driving with cone pose
        // if holding a cube enter driving with cube pose
    }

    @Override
    public boolean isFinished() {
        // FIXME: do everything
        // if not holding a piece true
        if (!armSubsystem.isGamePieceDetected()) {
            return true;
        }
        // otherwise keep running until interrupted
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // FIXME: do everything
        if (interrupted) {
            // do nothing but log
        }
        else {
            // still do nothing but log
        }
        // print state when ended
    }
}
