package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase{

    private final double shooterSpeed;

    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    private long shotStartTime = 0;

    private final float KICKER_SPEED = 0.75f;

    private final int EXCECUTE_TIME_SECONDS = 3;
    private final int FINISH_TIME_SECONDS = 4;

    private final int MILLISECONDS_TO_SECONDS = 1000;

    public ShootCommand(double shooterSpeed, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {

        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSpeed    = shooterSpeed;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Start the shooter motor and capture the start time
        shooterSubsystem.setShooterMotorSpeed(shooterSpeed);
        shotStartTime = System.currentTimeMillis();

        // Move the rollers out of the way of the shot
        intakeSubsystem.deployRollerArm();
    }

    // Called every time the scheduler runs while the command is scheduled.
    // shooter will start and then kicker should start 0.5 seconds after
    // button #2 = B on controller
    @Override
    public void execute() {

        if (System.currentTimeMillis() - shotStartTime > (EXCECUTE_TIME_SECONDS * MILLISECONDS_TO_SECONDS)) {
            shooterSubsystem.setKickerMotorSpeed(KICKER_SPEED);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
        intakeSubsystem.retractRollerArm();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        //return (System.currentTimeMillis() - shotStartTime > finishTime) ? true : false;

        // Stop the shooter after a set time
        if (System.currentTimeMillis() - shotStartTime > (FINISH_TIME_SECONDS * MILLISECONDS_TO_SECONDS)) {
            return true;
        }

        return false;
    }

}
