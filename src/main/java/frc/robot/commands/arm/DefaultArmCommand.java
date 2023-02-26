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
        if (driverController.getBackButton()) {
            armSubsystem.setArmLiftEncoder(0);
            armSubsystem.setArmExtendEncoder(0);
            armSubsystem.setPincherEncoder(0);
        }

        // POV controls the arm up down extend/retract
        int armMovement = driverController.getPOV();

        switch (armMovement) {

        case 0: // up (against gravity)
            armSubsystem.setArmLiftSpeed(.1);
            break;

        case 90: // right
            armSubsystem.setArmExtendSpeed(.1);
            break;

        case 180: // down (gravity is helping)
            armSubsystem.setArmLiftSpeed(-.03);
            break;

        case 270: // left
            armSubsystem.setArmExtendSpeed(-.1);
            break;

        default:
            // no relevant button pressed
            armSubsystem.setArmLiftSpeed(0); // NOTE: arm will gradually fall
            armSubsystem.setArmExtendSpeed(0);
            break;
        }

        // Pincher arm movement
        // NOTE: we are out of buttons on the controller, so use the
        // diagonal POV to open/close the pincher
        switch (armMovement) {

        case 45: // open pincher
            armSubsystem.setPincherSpeed(.3);
            break;

        case 315: // close pincher
            armSubsystem.setPincherSpeed(-.5);
            break;

        default:
            armSubsystem.setPincherSpeed(0);
            break;
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
            System.out.println("DefaultArmCommand interrupted.");
        }
        else {
            System.out.println("DefaultArmCommand ended.");
        }
    }
}