package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;

public class CalibratePincherCommand extends BaseArmCommand {


    public CalibratePincherCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);
        addRequirements(armSubsystem);

    }

    @Override
    public void initialize() {

        logCommandStart();
    }

    @Override
    public void execute() {

        // raise arm and open claw
        if (moveArmLiftToAngle(40)) {
            openPincher();
        }

    }

    @Override
    public boolean isFinished() {

        if (armSubsystem.isPincherOpen()) {
            setFinishReason("Pincher open detected");
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {

        stopArmMotors();

        logCommandEnd(interrupted);
    }

}