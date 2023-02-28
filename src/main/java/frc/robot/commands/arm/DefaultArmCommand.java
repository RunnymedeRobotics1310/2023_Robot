package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.operator.Runnymede2023GameController;
import frc.robot.subsystems.ArmSubsystem;

public class DefaultArmCommand extends CommandBase {

    private final ArmSubsystem                armSubsystem;
    private final Runnymede2023GameController driverController;

    /**
     * Creates a new ExampleCommand.
     *
     * @param ArmSubsystem The subsystem used by this command.
     */
    public DefaultArmCommand(Runnymede2023GameController driverController, ArmSubsystem armSubsystem) {

        this.driverController = driverController;
        this.armSubsystem     = armSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("DefaultArmCommand started.");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // zero the encoders if required
        if (driverController.isArmReset()) {
            armSubsystem.setArmLiftEncoder(0);
            armSubsystem.setArmExtendEncoder(0);
            armSubsystem.setPincherEncoder(0);
        }

        armSubsystem.setArmLiftSpeed(driverController.getArmLiftMotorSpeed());
        armSubsystem.setArmExtendSpeed(driverController.getArmExtendMotorSpeed());
        armSubsystem.setPincherSpeed(driverController.getPincerMotorSpeed());

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
            System.out.println("DefaultArmCommand interrupted.");
        }
        else {
            System.out.println("DefaultArmCommand ended.");
        }
    }
}