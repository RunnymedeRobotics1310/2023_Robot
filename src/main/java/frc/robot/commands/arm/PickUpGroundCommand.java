package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.commands.operator.DriverController;
import frc.robot.subsystems.ArmSubsystem;

public class PickUpGroundCommand extends CommandBase {

    private final ArmSubsystem     armSubsystem;
    private final DriverController driverController;
    private final double           GROUND_PICKUP_ARM_HEIGHT = 2;

    // fixme: do everything - see table
    // https://docs.google.com/document/d/1JzU-BzCXjGCwosouylmWGN83-x8lv-oPzklcXDqNN2U/edit#

    public PickUpGroundCommand(DriverController driverController, ArmSubsystem armSubsystem) {

        this.armSubsystem     = armSubsystem;
        this.driverController = driverController;

        addRequirements(armSubsystem);

    }

    @Override
    public void initialize() {

        System.out.println("PickUpGroundCommand started");
        // dump arm data
    }

    @Override
    public void execute() {

        // sets arm position to ground pickup
        // FIXME: convert to using math.abs instead of raw values
        if (GROUND_PICKUP_ARM_HEIGHT < armSubsystem.getArmLiftEncoder()) {
            armSubsystem.setArmLiftSpeed(1);
        }
        else if (GROUND_PICKUP_ARM_HEIGHT > armSubsystem.getArmLiftEncoder()) {
            armSubsystem.setArmLiftSpeed(-1);
        }
        else {
            armSubsystem.setArmLiftSpeed(0);
        }

        // pick up cube or cone depending on controller input when piece is detected
        // FIXME: make sure this is correct, convert to using math.abs instead of raw values
        if (armSubsystem.isGamePieceDetected()) {
            if (driverController.isPickUpCube() && GamePiece.CUBE.pincherEncoderCount < armSubsystem.getPincherEncoder()) {
                armSubsystem.setPincherSpeed(1);
            }
            else if (driverController.isPickUpCone() && GamePiece.CONE.pincherEncoderCount < armSubsystem.getPincherEncoder()) {
                armSubsystem.setPincherSpeed(1);
            }
            else {
                armSubsystem.setPincherSpeed(0);
            }

        }

    }

    @Override
    public boolean isFinished() {
        // FIXME: set cube or cone, return false if no piece

        if (driverController.isPickUpCube()) {
            // set state to cube
            return true;
        }
        else if (driverController.isPickUpCone()) {
            // set state to cone
            return true;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // FIXME: do everything
        // dump arm data
    }
}
