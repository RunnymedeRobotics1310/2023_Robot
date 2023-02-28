package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class PickUpConeCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final ArmSubsystem   armSubsystem;

    // fixme: do everything - see table https://docs.google.com/document/d/1JzU-BzCXjGCwosouylmWGN83-x8lv-oPzklcXDqNN2U/edit#

    public PickUpConeCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {

        this.driveSubsystem = driveSubsystem;
        this.armSubsystem   = armSubsystem;

        addRequirements(driveSubsystem, armSubsystem);

    }

    @Override
    public void initialize() {

        System.out.println("PickUpConeCommand started");

    }

    @Override
    public void execute() {

        // FIXME: do everything

        ;

    }

    @Override
    public boolean isFinished() {
        // fixme: do everything
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // fixme: do everything
    }
}
