package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.commands.operator.RunnymedeGameController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DefaultDriveCommand extends CommandBase {

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
    public DefaultDriveCommand(RunnymedeGameController driverController, DriveSubsystem driveSubsystem,
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

        switch (driveMode) {
        case ARCADE:
            setMotorSpeedsArcade();
            break;
        case TANK:
            setMotorSpeedsTank();
            break;
        case QUENTIN:
            setMotorSpeedsQuentin();
            break;
        default:
            setMotorSpeedsQuentin();
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
        // Can delete if we don't need a deadstop
        // Otherwise must be fixed
        // if (driverController.getLeftBumper()) {
        // driveSubsystem.setMotorSpeeds(0, 0);
        // }
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

        // Filter out low input values to reduce drivetrain drift
        double leftY      = driverController.getRawAxis(1);
        double leftX      = driverController.getRawAxis(0);
        double leftSpeed  = leftY + leftX / (leftY == 0 ? 1 : 2); // less sensitive when moving
        double rightSpeed = leftY - leftX / (leftY == 0 ? 1 : 2);

        // Boost
        if (driverController.getRightBumper()) {
            driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
        }
        else {
            driveSubsystem.setMotorSpeeds(leftSpeed / 2, rightSpeed / 2);
        }
    }

    private void setMotorSpeedsTank() {

        double leftSpeed  = driverController.getRawAxis(1);
        double rightSpeed = driverController.getRawAxis(5);

        // Boost
        if (driverController.getRightBumper()) {
            driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
        }
        else {
            driveSubsystem.setMotorSpeeds(leftSpeed / 2, rightSpeed / 2);
        }
    }

    private static final int AXIS_LEFT_Y  = 1;
    private static final int AXIS_RIGHT_X = 4;

    private void setMotorSpeedsQuentin() {

        // forwards/backwards speed
        double       speed   = driverController.getRawAxis(AXIS_LEFT_Y);
        // turn speed
        final double rawTurn = driverController.getRawAxis(AXIS_RIGHT_X);

        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putNumber("Turn", rawTurn);

        double  turn      = rawTurn / 2;
        boolean boost     = driverController.getRightBumper();

        double  leftSpeed = 0, rightSpeed = 0;

        if (!boost) {
            speed = speed / 2;
        }
        else {
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

            // Turning left
            if (rawTurn < 0) {

                // If boosted and at full speed, and the
                // turn is limited to 0.5, then the max turn
                // that can be achieved is 1.0 vs 0.5.
                leftSpeed  = speed + turn;
                rightSpeed = speed;
            }
            // Turning right
            else if (rawTurn > 0) {
                leftSpeed  = speed;
                rightSpeed = speed - turn;
            }
            else {
                leftSpeed  = speed;
                rightSpeed = speed;
            }
        }
        else {
            if (rawTurn < 0) {
                leftSpeed  = speed;
                rightSpeed = speed - turn;
            }
            else if (rawTurn > 0) {
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
}