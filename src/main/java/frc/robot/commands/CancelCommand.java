package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This command is used to safely stop the robot in its current position, and to cancel any running
 * commands
 */
public class CancelCommand extends RunnymedeCommand {

    private final DriveSubsystem  driveSubsystem;
    private final ArmSubsystem    armSubsystem;
    private final VisionSubsystem visionSubsystem;

    private long                  startTime = 0;

    /**
     * Cancel the commands running on all subsystems.
     *
     * All subsystems must be passed to this command, and each subsystem should have a stop command
     * that safely stops the robot
     * from moving.
     */
    public CancelCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem) {

        this.driveSubsystem  = driveSubsystem;
        this.armSubsystem    = armSubsystem;
        this.visionSubsystem = visionSubsystem;

        addRequirements(driveSubsystem, armSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {

        logCommandStart();

        driveSubsystem.stop();
        armSubsystem.stop();
        visionSubsystem.stop();

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
