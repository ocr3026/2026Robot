package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeIOSpark implements IntakeIO {
    private final SparkFlex intakeMotor;
    private final SparkClosedLoopController intakePID;
    private final SparkMax intakeLift;
    private final SparkClosedLoopController intakeLiftPID;

    public IntakeIOSpark() {
        intakeMotor = new SparkFlex(IntakeConstants.intakeMotorID, MotorType.kBrushless);
        intakeLift = new SparkMax(IntakeConstants.intakeLiftID, MotorType.kBrushless);
        intakePID = intakeMotor.getClosedLoopController();
        intakeLiftPID = intakeLift.getClosedLoopController();
        SparkFlexConfig intakeConfig = new SparkFlexConfig();
        SparkMaxConfig intakeLiftConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeLiftConfig.idleMode(IdleMode.kBrake);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeAppliedVolts = intakeMotor.getAppliedOutput();
        inputs.intakeAppliedVolts = intakeLift.getAppliedOutput();
        inputs.intakeConnected = true;
        inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
        inputs.intakeCurrentAmps = intakeLift.getOutputCurrent();
        inputs.intakePosition = Degrees.of(intakeMotor.getEncoder().getPosition());
        inputs.intakePosition = Degrees.of(intakeLift.getEncoder().getPosition());
        inputs.intakeVelocity = DegreesPerSecond.of(intakeMotor.getEncoder().getVelocity());
        inputs.intakeVelocity = DegreesPerSecond.of(intakeLift.getEncoder().getVelocity());  
    }

    @Override
    public void setIntakeSpeed(double speed) {
        intakePID.setSetpoint(speed, ControlType.kVoltage);
    }

     @Override
    public void setIntakeLiftSpeed(double speed) {
        intakeLiftPID.setSetpoint(speed, ControlType.kVoltage);
    }
}


