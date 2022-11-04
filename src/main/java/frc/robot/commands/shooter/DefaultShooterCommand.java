package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;


public class DefaultShooterCommand extends CommandBase {

    private final ShooterSubsystem shooterSubsystem;
    private final XboxController driverController;
	private boolean shooterOn = false;
	private boolean previousState = false;
	private long shooterStartTime = 0;

    public DefaultShooterCommand(XboxController driverController, ShooterSubsystem shooterSubsystem) {
        this.driverController = driverController;
		this.shooterSubsystem = shooterSubsystem;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(shooterSubsystem);
    }   

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

	// Called every time the scheduler runs while the command is scheduled.
	// shooter will start and then kicker should start 0.5 seconds after
	// button #2 = B on controller
	@Override
	public void execute() {
		// if(driverController.getRawButton(2) && !previousState){
		// 	previousState = true;
		// 	shooterOn = !shooterOn;
		// 	shooterStartTime = System.currentTimeMillis();
		// }
		// else if(!driverController.getRawButton(2)){
		// 	previousState = false;
		// }

		// if(shooterOn) {
		// 	shooterSubsystem.setShooterMotorSpeed(0.3);
		// 	if (System.currentTimeMillis() - shooterStartTime > 1000){
		// 		shooterSubsystem.setKickerMotorSpeed(0.3);
		// 	} 
		// 	// if (!ballSensor.get()){
		// 	// 	shooterOn = !shooterOn;
		// 	// }
		// }
		// else{
		// 	shooterSubsystem.setShooterMotorSpeed(0);
		// 	shooterSubsystem.setKickerMotorSpeed(0);
		// }
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

}
