package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetGyroPitchCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;

    /**
     * Set the current heading in the driveSubsystem
     *
     * @param heading value 0-360
     * @param driveSubsystem
     */
    public ResetGyroPitchCommand(DriveSubsystem driveSubsystem) {

        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {

        System.out.println("Reset the gyro pitch to zero");

        driveSubsystem.setGyroPitch(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
