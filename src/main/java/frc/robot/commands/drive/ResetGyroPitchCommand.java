package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ResetGyroPitchCommand extends InstantCommand {

    public ResetGyroPitchCommand(DriveSubsystem driveSubsystem) {

        super(() -> {

            System.out.println("ResetGyroPitchCommand: Reset the gyro pitch to zero");

            driveSubsystem.setGyroPitch(0);
        });
    }

    @Override
    public boolean runsWhenDisabled() {
        // Allow the pitch to be reset when disabled
        return true;
    }

}
