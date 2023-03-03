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

        double tolerance = -2;
        // Q is working on this

        // ensure that we have a piece
        if (armSubsystem.getHeldGamePiece() == GamePiece.NONE) {
            System.out.print("Can not score high, no piece is held.");
            return;
        }
        if (armSubsystem.getHeldGamePiece() == GamePiece.CUBE) {
            // Cube hight & extend
            if (armSubsystem.getArmLiftEncoder() - ArmConstants.TOP_CUBE_HEIGHT <= tolerance) {
                armSubsystem.setArmLiftSpeed(.25);
            }
            else if (armSubsystem.getArmLiftEncoder() - ArmConstants.TOP_CUBE_HEIGHT >= tolerance) {
                armSubsystem.setArmLiftSpeed(-.25);
            }
            else {
                armSubsystem.setArmLiftSpeed(0);
            }

            if (armSubsystem.getArmLiftEncoder() - ArmConstants.TOP_CUBE_EXTEND <= tolerance) {
                armSubsystem.setArmLiftSpeed(.25);
            }
            else if (armSubsystem.getArmLiftEncoder() - ArmConstants.TOP_CUBE_EXTEND >= tolerance) {
                armSubsystem.setArmLiftSpeed(-.25);
            }
            else {
                armSubsystem.setArmLiftSpeed(0);
            }

        }
        else if (armSubsystem.getHeldGamePiece() == GamePiece.CONE) {
            // Cone height & extend
            if (armSubsystem.getArmLiftEncoder() - ArmConstants.TOP_CONE_HEIGHT <= tolerance) {
                armSubsystem.setArmLiftSpeed(.25);
            }
            else if (armSubsystem.getArmLiftEncoder() - ArmConstants.TOP_CONE_HEIGHT >= tolerance) {
                armSubsystem.setArmLiftSpeed(-.25);
            }
            else {
                armSubsystem.setArmLiftSpeed(0);
            }

            if (armSubsystem.getArmLiftEncoder() - ArmConstants.TOP_CONE_EXTEND <= tolerance) {
                armSubsystem.setArmLiftSpeed(.25);
            }
            else if (armSubsystem.getArmLiftEncoder() - ArmConstants.TOP_CONE_EXTEND >= tolerance) {
                armSubsystem.setArmLiftSpeed(-.25);
            }
            else {
                armSubsystem.setArmLiftSpeed(0);
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
