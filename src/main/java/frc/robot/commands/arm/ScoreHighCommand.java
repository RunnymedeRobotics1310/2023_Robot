package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants.CameraView;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionTargetType;

public class ScoreHighCommand extends CommandBase {

    private final ArmSubsystem    armSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final DriveSubsystem  driveSubsystem;

    // fixme: do everything - see table
    // https://docs.google.com/document/d/1JzU-BzCXjGCwosouylmWGN83-x8lv-oPzklcXDqNN2U/edit#

    public ScoreHighCommand(ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {

        this.armSubsystem    = armSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.driveSubsystem  = driveSubsystem;

        addRequirements(armSubsystem, visionSubsystem, driveSubsystem);

    }

    @Override
    public void initialize() {

        System.out.println("ScoreHighCommand started");

        armSubsystem.setArmLiftSpeed(0);
        armSubsystem.setArmExtendSpeed(0);
        armSubsystem.setPincherSpeed(0);
        driveSubsystem.setMotorSpeeds(0, 0);

        visionSubsystem.setCameraView(CameraView.HIGH);

    }

    @Override
    public void execute() {

        // FIXME: do everything

        // ask arm if it thinks its holding a piece
        // set limelignt to april tags (cube) or tape (cone)
        if (armSubsystem.getPincherEncoder() == /* ArmSubsystem.PincherContents.CUBE.width */ 0) {
            visionSubsystem.setVisionTargetType(VisionTargetType.TAG);
        }
        else if (armSubsystem.getPincherEncoder() == /* ArmSubsystem.PincherContents.CONE.width */ 0) {
            visionSubsystem.setVisionTargetType(VisionTargetType.CONE_POST);
        }
        else {
            System.out.print("You need to hold a piece");
        }

        // drive to target

        // pose: score high
        // raise arm
        // extend arm

        // finish



        ;

    }

    @Override
    public boolean isFinished() {
        // FIXME: do everything
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
