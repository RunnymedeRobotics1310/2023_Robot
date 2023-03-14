package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;

public class CalibratePincherCommand extends BaseArmCommand {


    public CalibratePincherCommand(ArmSubsystem armSubsystem) {
        super(armSubsystem);
        addRequirements(armSubsystem);

    }

    @Override
    public void initialize() {
        System.out.print("InitializeArmCommand started");
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
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {

        stopArmMotors();

        if (interrupted) {
            System.out.print("CalibratePincherCommand interrupted");
        }
        else {
            System.out.print("CalibratePincherCommand ended");
        }
        printArmState();
    }

}