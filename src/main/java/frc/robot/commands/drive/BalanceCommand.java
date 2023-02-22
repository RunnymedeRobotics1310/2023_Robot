

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    long                         startTime = 0;

    // Track the gyro pitch.
    double                       pitch     = 0;
    double                       pitchRate = 0;
    double                       speed     = 0;

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
        pitch     = driveSubsystem.getPitch();
        pitchRate = driveSubsystem.getPitchRate();


        // if (Math.abs(pitchRate) > 1) {
        // speed = 0;
        // }
        if (pitch > 1) {
            speed = .15;
        }
        else if (pitch < -1) {
            speed = -.15;
        }
        else {
            speed = 0;
        }

        driveSubsystem.setMotorSpeeds(speed, speed);

    }

    @Override
    public boolean isFinished() {

        // Track the gyro pitch.
        pitch     = driveSubsystem.getPitch();
        pitchRate = driveSubsystem.getPitchRate();

        // FIXME: Only finish when it has been still for a couple of seconds. Instantaneous 0 is too
        // soon, because the charger may be in the middle of passing through 0 while rocking.
        if (pitch < 0.5 && pitch > -0.5) {
            return true;
        }
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
