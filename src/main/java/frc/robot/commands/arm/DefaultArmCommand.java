package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;

public class DefaultArmCommand extends BaseArmCommand {

    private final OperatorInput operatorInput;

    /**
     * Creates a new ExampleCommand.
     *
     * @param ArmSubsystem The subsystem used by this command.
     */
    public DefaultArmCommand(OperatorInput driverController, ArmSubsystem armSubsystem) {

        super(armSubsystem);

        this.operatorInput = driverController;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double armAngleLiftIncrement = operatorInput.getArmLiftMotorSpeed() * ArmConstants.MAX_ARM_ANGLE_LIFT_LOOP_INCREMENT;

        // Lift up faster than down
        if (armAngleLiftIncrement > 0) {
            armAngleLiftIncrement *= 1.5;
        }

        armSubsystem.moveArmLiftToAngle(armSubsystem.getArmLiftAngleSetpoint() + armAngleLiftIncrement);

        armSubsystem.setPincherSpeed(operatorInput.getPincherMotorSpeed());

        // If the arm is down, keep the arm retracted
        if (armSubsystem.isArmDown()) {
            retractArm();
        }
        else {
            armSubsystem.setArmExtendSpeed(operatorInput.getArmExtendMotorSpeed());
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
        logCommandEnd(interrupted);
    }
}