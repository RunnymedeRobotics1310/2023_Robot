package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class SetGyroHeadingCommand extends InstantCommand {

    /**
     * Set the current heading in the driveSubsystem
     *
     * @param heading value 0-360
     * @param driveSubsystem
     */
    public SetGyroHeadingCommand(double heading, DriveSubsystem driveSubsystem) {

        super(() -> {

            System.out.println("SetGyroHeadingCommand: Set the current heading to " + heading);

            driveSubsystem.setGyroHeading(heading);
        });
    }

    @Override
    public boolean runsWhenDisabled() {
        // Allow the gyro heading to be set when the robot is disabled
        return true;
    }

}
