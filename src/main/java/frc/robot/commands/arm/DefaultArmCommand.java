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

        /*
         * POV controls the arm up down extend/retract
         * in combination with the triggers.
         *
         * The POV selects which motor will move, and the
         * Triggers move the motor.
         *
         * UP = extend/retract arm
         * LEFT = open/close pincher
         * DOWN = lift/lower arm
         */

        int    armMotorSelect = driverController.getPOV();
        double motorSpeed     = 0;

        // If both the left and right triggers are pressed then
        // do not move the motors.
        if (driverController.getLeftTriggerAxis() > 0
            && driverController.getRightTriggerAxis() > 0) {

            motorSpeed = 0;
        }
        else if (driverController.getLeftTriggerAxis() > 0) {

            motorSpeed = -driverController.getLeftTriggerAxis();
        }
        else if (driverController.getRightTriggerAxis() > 0) {

            motorSpeed = driverController.getRightTriggerAxis();
        }


        switch (armMotorSelect) {

        case 0:
            armSubsystem.setArmLiftSpeed(0);
            armSubsystem.setArmExtendSpeed(motorSpeed);
            armSubsystem.setPincherSpeed(0);
            break;

        case 180:
            armSubsystem.setArmLiftSpeed(motorSpeed);
            armSubsystem.setArmExtendSpeed(0);
            armSubsystem.setPincherSpeed(0);
            break;

        case 270: // left
            armSubsystem.setArmLiftSpeed(0);
            armSubsystem.setArmExtendSpeed(0);
            armSubsystem.setPincherSpeed(motorSpeed);
            break;

        default:
            // no relevant button pressed
            armSubsystem.setArmLiftSpeed(0);
            armSubsystem.setArmExtendSpeed(0);
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