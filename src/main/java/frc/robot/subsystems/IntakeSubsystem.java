package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    // The motors on the top part of the intake thing.
    private final VictorSPX intakeRollerMotor =
            new VictorSPX(IntakeConstants.INTAKE_MOTOR_ADDRESS);

    private double intakeRollerMotorSetpoint = 0;

    /** The roller piston is used to deploy the roller arm */
    private DoubleSolenoid intakeRollerPiston = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            IntakeConstants.INTAKE_ROLLER_PISTON_ADDRESS,
            IntakeConstants.INTAKE_ROLLER_PISTON_ADDRESS + 1);

    Value intakeRollerValue = Value.kReverse;

    private DoubleSolenoid intakeHoodPiston = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            IntakeConstants.INTAKE_HOOD_PISTON_ADDRESS,
            IntakeConstants.INTAKE_HOOD_PISTON_ADDRESS + 1);

    Value intakeHoodValue = Value.kReverse;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {

        intakeRollerMotor.setInverted(IntakeConstants.INTAKE_MOTOR_REVERSED);

        // Default to intake retracted
        retractRollerArm();
        closeHood();
    }

    /** Open the intake hood to take in a ball */
    public void openHood() {
        intakeHoodValue = Value.kReverse;
        intakeHoodPiston.set(intakeHoodValue);
    }

    /** Close the intake hood */
    public void closeHood() {
        intakeHoodValue = Value.kForward;
        intakeHoodPiston.set(intakeHoodValue);
    }

    /** Deploy the intake roller arm */
    public void deployRollerArm() {
        intakeRollerValue = Value.kReverse;
        intakeRollerPiston.set(intakeRollerValue);

    }

    /** Retract the intake roller arm */
    public void retractRollerArm() {
        intakeRollerValue = Value.kForward;
        intakeRollerPiston.set(intakeRollerValue);
    }

    /**
     * Set the speed on the intake motor
     * @param speed in the range -1.0 to 1.0, 0 = stopped;
     */
    public void setIntakeRollerSpeed(double speed) {
        intakeRollerMotorSetpoint = speed;
        intakeRollerMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Motor",  intakeRollerMotorSetpoint);
        SmartDashboard.putString("Intake Hood",   intakeHoodValue.toString());
        SmartDashboard.putString("Intake Roller", intakeRollerValue.toString());
    }
}
