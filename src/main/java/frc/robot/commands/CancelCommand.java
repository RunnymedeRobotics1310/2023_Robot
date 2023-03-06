package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This command is used to safely stop the robot in its current position, and to
 * cancel any running commands
 */
public class CancelCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final ArmSubsystem   armSubsystem;

    private long                 startTime = 0;

    /**
     * Cancel the commands running on all subsystems.
     *
     * All subsystems must be passed to this command, and each subsystem should have a stop command
     * that safely stops the robot from moving.
     */
    public CancelCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {

        this.driveSubsystem = driveSubsystem;
        this.armSubsystem   = armSubsystem;

        addRequirements(driveSubsystem, armSubsystem);
    }

    @Override
    public void initialize() {

        System.out.println("Cancel Command started.");

        driveSubsystem.stop();
        armSubsystem.stop();

        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {

        // Wait 1/4 second before finishing.
        // Allow time for the robot to stop moving
        if (System.currentTimeMillis() - startTime > 250) {
            return true;
        }

        return false;
    }

}
