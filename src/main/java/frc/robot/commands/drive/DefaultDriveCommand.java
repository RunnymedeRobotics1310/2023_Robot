package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DefaultDriveCommand extends CommandBase {
    private static final double     DRIVE_FILTER_VALUE = 0.2;
    private final DriveSubsystem    driveSubsystem;
    private final XboxController    driverController;
    private final DriveModeSelector driveModeSelector;

    // FIXME: Why is the vision subsystem being passed to the default drive command.
    // What are we expecting to do with the vision subsystem?
    // Please document any ideas here....
    private final VisionSubsystem   visionSubsystem;


    /**
     * Creates a new ExampleCommand.
     *
     * @param driveSubsystem The subsystem used by this command.
     */
    public DefaultDriveCommand(XboxController driverController, DriveSubsystem driveSubsystem,
        DriveModeSelector driveModeSelector, VisionSubsystem visionSubsystem) {

        this.driverController  = driverController;
        this.driveSubsystem    = driveSubsystem;
        this.driveModeSelector = driveModeSelector;
        this.visionSubsystem   = visionSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("DefaultDriveCommand started.");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        DriveMode driveMode = driveModeSelector.getDriveMode();

        double    speed     = 0, turn = 0;

        switch (driveMode) {

        case ARCADE:
            // FIXME: We could use Quentin for single stick arcade by
            // passing in the appropriate speed and turn values from
            // the different sticks
            setMotorSpeedsArcade();
            break;

        case TANK:
            setMotorSpeedsTank();
            break;

        case DUAL_STICK_ARCADE:

            // Left Y is speed, right X is turn
            speed = getScaledValue(-driverController.getLeftY());
            turn = getScaledValue(driverController.getRightX()) / 2;

            setMotorSpeedsDualStickArcade(speed, turn);
            break;

        case QUENTIN: // Quentin is the default drive mode.
        default:

            // Left Y is speed, right X is turn
            speed = getScaledValue(-driverController.getLeftY());
            turn = getScaledValue(driverController.getRightX()) / 2;

            setMotorSpeedsQuentin(speed, turn);

            break;
        }

        //
        // handle dead stop
        //
        // FIXME: The placement of this code may not result in the expected action
        //
        // The execute loop is called 50 times per second.
        //
        // The code above sets the motor speeds to a value, and then dead stop code
        // sets the motor speeds to a different value.
        //
        // The "dead stop" function may not actually stop the robot, it may
        // cause a jerky motor operation where the motors are constantly being
        // turned on and off again.
        //
        // Possible Solution: The dead stop code needs to take priority
        // over all other setting of the motor speeds. If there is a dead stop,
        // no other setting of the motor speeds should be possible.
        //
        // NOTE: the bumper dead stop feature overrides the joysticks temporarily
        // while held. When the bumper is released, the joysticks will take over again.
        // Hopefully that is the intended design.
        //
        if (driverController.getLeftBumper()) {
            driveSubsystem.setMotorSpeeds(0, 0);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("DefaultDriveCommand interrupted.");
        }
        else {
            System.out.println("DefaultDriveCommand ended.");
        }
    }

    private void setMotorSpeedsArcade() {

        // TODO: Filtering of joystick values to handle the deadband should be done in a
        // joystick
        // class
        /** see {@link RunnymedeGameController}. */

        // Filter out low input values to reduce drivetrain drift
        double leftY      = getScaledValue(driverController.getLeftY());
        double leftX      = getScaledValue(driverController.getLeftX());
        double leftSpeed  = leftY * -1 + leftX;
        double rightSpeed = leftY * -1 - leftX;

        // Boost
        if (driverController.getRightBumper()) {
            driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
        }
        else {
            driveSubsystem.setMotorSpeeds(leftSpeed / 2, rightSpeed / 2);
        }
    }

    private void setMotorSpeedsTank() {

        double leftSpeed  = getScaledValue(-driverController.getLeftY());
        double rightSpeed = getScaledValue(-driverController.getRightY());

        // Boost
        if (driverController.getRightBumper()) {
            driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
        }
        else {
            driveSubsystem.setMotorSpeeds(leftSpeed / 2, rightSpeed / 2);
        }
    }

    private void setMotorSpeedsQuentin(double speed, double turn) {

        double  turn2 = turn;
        boolean boost = driverController.getRightBumper();

        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putNumber("Turn", turn);

        double leftSpeed = 0, rightSpeed = 0;

        if (!boost) {
            speed = speed / 2;
        }
        else {
            // Set the speed as full forward +1.0 or backwards -1.0
            speed = Math.signum(speed);
        }

        if (speed < 0) {
            turn = -turn;
        }

        if (speed == 0) {
            leftSpeed  = turn;
            rightSpeed = -turn;
        }
        else if (boost) {

            if (turn2 < 0) {
                leftSpeed  = speed + turn;
                rightSpeed = speed;
            }
            else if (turn2 > 0) {
                leftSpeed  = speed;
                rightSpeed = speed - turn;
            }
            else {
                leftSpeed  = speed;
                rightSpeed = speed;
            }
        }
        else {
            if (turn2 < 0) {
                leftSpeed  = speed;
                rightSpeed = speed - turn;
            }
            else if (turn2 > 0) {
                leftSpeed  = speed + turn;
                rightSpeed = speed;
            }
            else {
                rightSpeed = speed;
                leftSpeed  = speed;
            }
        }
        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
    }

    private void setMotorSpeedsDualStickArcade(double speed, double turn) {

        boolean boost     = driverController.getRightBumper();

        double  leftSpeed = 0, rightSpeed = 0, max = 1.0;

        if (!boost) {
            speed /= 2;
            turn  /= 2;
            max   /= 2;
        }

        // If there is no forward movement, then spin on the spot
        if (speed == 0) {
            leftSpeed  = turn;
            rightSpeed = -turn;
        }
        else {

            if (Math.abs(speed) + Math.abs(turn) <= max) {

                // If the speed + turn is less than the max, then apply the turn equally to
                // the left and right side.
                leftSpeed  = speed + turn;
                rightSpeed = speed - turn;
            }
            else {

                // If the speed + turn > 1 then we need to limit the one
                // side of the robot to a speed of 1.0, and maintain the differential
                // between the two sides (2 * turn).

                if (speed > 0) {

                    // Forward movement

                    if (turn > 0) {

                        leftSpeed  = max;
                        rightSpeed = max - 2 * turn;
                    }
                    else {

                        leftSpeed  = max + 2 * turn; // -ve turn value
                        rightSpeed = max;
                    }

                }

                else {

                    // Backwards movement

                    if (turn > 0) {

                        leftSpeed  = -max + 2 * turn;
                        rightSpeed = -max;
                    }
                    else {

                        leftSpeed  = -max;
                        rightSpeed = -max - 2 * turn; // -ve turn value

                    }
                }
            }
        }

        // Apply the calculated speed to the motors
        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
    }

    private static double getScaledValue(double value) {

        if (Math.abs(value) < DRIVE_FILTER_VALUE) {
            value = 0;
        }
        else {
            // Scale the range of [0.2-1.0] to [0.0-1.0];
            value = ((Math.abs(value) - DRIVE_FILTER_VALUE) / (1 - DRIVE_FILTER_VALUE)) * Math.signum(value);
        }
        return value;
    }
}