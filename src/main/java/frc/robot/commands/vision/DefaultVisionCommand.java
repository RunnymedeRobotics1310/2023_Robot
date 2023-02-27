package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.operator.Runnymede2023GameController;
import frc.robot.subsystems.VisionSubsystem;

public class DefaultVisionCommand extends CommandBase {

    private final VisionSubsystem             visionSubsystem;
    private final Runnymede2023GameController driverController;

    /**
     * Default Vision Command.
     *
     * @param driverController
     * @param visionSubsystem
     */
    public DefaultVisionCommand(Runnymede2023GameController driverController,
        VisionSubsystem visionSubsystem) {

        this.driverController = driverController;
        this.visionSubsystem  = visionSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(visionSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("DefaultVisionCommand started.");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // zero the encoders if required
        if (driverController.getBackButton()) {
            visionSubsystem.setCameraEncoderPosition(0);
        }

        /*
         * POV controls the Vision Camera angle.
         *
         * When POV right is selected the triggers move the camera
         */

        boolean visionMotorSelected = driverController.getPOV() == 90;
        double  motorSpeed          = 0;

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


        if (visionMotorSelected) {
            visionSubsystem.setCameraMotorSpeed(motorSpeed);
        }
        else {
            visionSubsystem.setCameraMotorSpeed(0);
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
            System.out.println("DefaultVisionCommand interrupted.");
        }
        else {
            System.out.println("DefaultVisionCommand ended.");
        }
    }
}