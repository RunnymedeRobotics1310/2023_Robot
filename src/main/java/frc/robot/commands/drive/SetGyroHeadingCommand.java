package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SetGyroHeadingCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;

    private final double         heading;

    /**
     * Set the current heading in the driveSubsystem
     *
     * @param heading value 0-360
     * @param driveSubsystem
     */
    public SetGyroHeadingCommand(double heading, DriveSubsystem driveSubsystem) {

        this.heading        = heading;
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {

        System.out.println("Set the current heading to " + heading);

        driveSubsystem.setGyroHeading(heading);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
