

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    long                         startTime = 0;

    /**
     * Drive on a specified compass heading (0-360 degrees) for the specified distance in cm.
     *
     * @param driveSubsystem
     */
    public BalanceCommand(DriveSubsystem driveSubsystem) {

        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);

    }

    @Override
    public void initialize() {

        System.out.println("BalanceCommand started at " + driveSubsystem.getPitch());
        startTime = System.currentTimeMillis();

    }

    @Override
    public void execute() {

        // Track the gyro pitch.
        double pitch     = driveSubsystem.getPitch();
        double pitchRate = driveSubsystem.getPitchRate();
        double speed     = 0;

        // if (Math.abs(pitchRate) > 1) {
        // speed = 0;
        // }
        if (pitch > 1) {
            speed = .15;
        }
        else if (pitch < -1) {
            speed = -.15;
        }

        driveSubsystem.setMotorSpeeds(speed, speed);

    }

    @Override
    public boolean isFinished() {

        if (System.currentTimeMillis() - startTime > 10000) {
            return true;
        }
        return false;

    }

    @Override
    public void end(boolean interrupted) {

        if (interrupted) {
            System.out.print("BalanceCommand interrupted");
        }
        else {
            System.out.print("BalanceCommand ended at " + driveSubsystem.getPitch());
        }

        // Stop the robot
        driveSubsystem.setMotorSpeeds(0, 0);
    }
}
