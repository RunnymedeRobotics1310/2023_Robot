

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    long startTime = 0;

    // Track the gyro pitch.
    double pitch         = 0;
    double pitchRate     = 0;
    double speed         = 0;
    long   prevLevelTime = 0;

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

    static final double ADJUST_SPEED = .02;

    @Override
    public void execute() {

        // Track the gyro pitch.
        pitch = driveSubsystem.getPitch();


        if (pitch > 1) {
            speed = ADJUST_SPEED;
        }
        else if (pitch < -1) {
            speed = -ADJUST_SPEED;
        }
        else {
            speed = 0;
        }

        // Feed forward balance \/
        speed = speed + pitch / 130;

        driveSubsystem.setMotorSpeeds(speed, speed);

    }

    @Override
    public boolean isFinished() {

        final double threshold        = 2;
        final long   levelAfterMillis = 350;

        // Timeout
        if (System.currentTimeMillis() - startTime > 10000) {
            return true;
        }
        // Track the gyro pitch.
        pitch = driveSubsystem.getPitch();

        // FIXME: Only finish when it has been still for a couple of seconds. Instantaneous 0 is too
        // soon, because the charger may be in the middle of passing through 0 while rocking.

        // Not level
        if (Math.abs(pitch) > threshold) {
            prevLevelTime = -1;
            return false;
        }
        // it's level!

        // first time?
        if (prevLevelTime == -1) {
            // yes - track time
            prevLevelTime = System.currentTimeMillis();
        }
        else {
            // it's already level - nothing to do.
        }

        if (System.currentTimeMillis() - prevLevelTime > levelAfterMillis) {
            // It's been level long enough!
            return true;
        }
        // Keep trying
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
