package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.subsystems.ArmSubsystem;

public class ScoreHighCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    // fixme: do everything - see table
    // https://docs.google.com/document/d/1JzU-BzCXjGCwosouylmWGN83-x8lv-oPzklcXDqNN2U/edit#

    public ScoreHighCommand(ArmSubsystem armSubsystem) {

        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);

    }

    @Override
    public void initialize() {

        System.out.println("ScoreHighCommand started");

        armSubsystem.setArmLiftSpeed(0);
        armSubsystem.setArmExtendSpeed(0);
        armSubsystem.setPincherSpeed(0);

    }

    @Override
    public void execute() {
        // Q is working on this

        // ensure that we have a piece
        if (armSubsystem.getHeldGamePiece() == GamePiece.NONE) {
            System.out.print("Can not score high, no piece is held.");
            return;
        }
        if (armSubsystem.getHeldGamePiece() == GamePiece.CUBE) {
            // Cube hight
            if (armSubsystem.getArmLiftEncoder() - ArmConstants.TOP_CUBE_HEIGHT <= ArmConstants.ARM_LIFT_MOTOR_TOLERANCE) {
                armSubsystem.setArmLiftSpeed(.25);
            }
            else if (armSubsystem.getArmLiftEncoder() - ArmConstants.TOP_CUBE_HEIGHT >= ArmConstants.ARM_LIFT_MOTOR_TOLERANCE) {
                armSubsystem.setArmLiftSpeed(-.25);
            }
            else {
                armSubsystem.setArmLiftSpeed(0);
            }
            // Cube extend
            if (armSubsystem.getArmLiftEncoder() - ArmConstants.TOP_CUBE_EXTEND <= ArmConstants.ARM_EXTEND_MOTOR_TOLERANCE) {
                armSubsystem.setArmLiftSpeed(.25);
            }
            else if (armSubsystem.getArmLiftEncoder() - ArmConstants.TOP_CUBE_EXTEND >= ArmConstants.ARM_EXTEND_MOTOR_TOLERANCE) {
                armSubsystem.setArmLiftSpeed(-.25);
            }
            else {
                armSubsystem.setArmLiftSpeed(0);
            }
            // Are we there yet?
            if (armSubsystem.getArmLiftEncoder() == ArmConstants.TOP_CUBE_HEIGHT
                && armSubsystem.getArmExtendEncoder() == ArmConstants.TOP_CONE_EXTEND) {
                return;
            }

        }
        else if (armSubsystem.getHeldGamePiece() == GamePiece.CONE) {
            // Cone height
            if (armSubsystem.getArmLiftEncoder() - ArmConstants.TOP_CONE_HEIGHT <= ArmConstants.ARM_LIFT_MOTOR_TOLERANCE) {
                armSubsystem.setArmLiftSpeed(.25);
            }
            else if (armSubsystem.getArmLiftEncoder() - ArmConstants.TOP_CONE_HEIGHT >= ArmConstants.ARM_LIFT_MOTOR_TOLERANCE) {
                armSubsystem.setArmLiftSpeed(-.25);
            }
            else {
                armSubsystem.setArmLiftSpeed(0);
            }
            // Cone extend
            if (armSubsystem.getArmLiftEncoder() - ArmConstants.TOP_CONE_EXTEND <= ArmConstants.ARM_EXTEND_MOTOR_TOLERANCE) {
                armSubsystem.setArmLiftSpeed(.25);
            }
            else if (armSubsystem.getArmLiftEncoder() - ArmConstants.TOP_CONE_EXTEND >= ArmConstants.ARM_EXTEND_MOTOR_TOLERANCE) {
                armSubsystem.setArmLiftSpeed(-.25);
            }
            else {
                armSubsystem.setArmLiftSpeed(0);
            }
            // Are we there yet?
            if (armSubsystem.getArmLiftEncoder() == ArmConstants.TOP_CONE_HEIGHT
                && armSubsystem.getArmExtendEncoder() == ArmConstants.TOP_CONE_EXTEND) {
                return;
            }

        }

    }

    @Override
    public boolean isFinished() {
        // FIXME: do everything
        // finish

        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // FIXME: do everything
        if (interrupted) {
            System.out.print("ScoreHighCommand interrupted");
        }
        else {
            System.out.print("ScoreHighCommand ended");
        }

    }
}
