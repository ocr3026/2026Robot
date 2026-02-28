package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
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

        FeedForwardConfig intakeConf = new FeedForwardConfig();
        intakeConf.kV(0.00245);

        SparkFlexConfig intakeConfig = new SparkFlexConfig();
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeConfig.closedLoop.p(0.0006).i(0).d(0.00192);
        intakeConfig.closedLoop.apply(intakeConf);

        FeedForwardConfig intakeLiftConf = new FeedForwardConfig();
        intakeLiftConf.kV(0.00245);
        
        SparkFlexConfig intakeLiftConfig = new SparkFlexConfig();
        intakeLiftConfig.idleMode(IdleMode.kBrake);
        intakeLiftConfig.closedLoop.p(0.0006).i(0).d(0.00192);
        intakeLiftConfig.closedLoop.apply(intakeLiftConf);

        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeLift.configure(intakeLiftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        zeroIntakeLift();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeAppliedVolts = intakeMotor.getAppliedOutput();
        inputs.intakeLiftAppliedVolts = intakeLift.getAppliedOutput();
        inputs.intakeConnected = true;
        inputs.intakeLiftConnected = true;
        inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
        inputs.intakeLiftCurrentAmps = intakeLift.getOutputCurrent();
        inputs.intakePosition = Degrees.of(intakeMotor.getEncoder().getPosition());
        inputs.intakeLiftPosition = Degrees.of(intakeLift.getEncoder().getPosition());
        inputs.intakeVelocity = DegreesPerSecond.of(intakeMotor.getEncoder().getVelocity());
        inputs.intakeLiftVelocity = DegreesPerSecond.of(intakeLift.getEncoder().getVelocity());  
    }

    @Override
    public void setIntakeSpeed(double speed) {
        intakePID.setSetpoint(speed, ControlType.kVelocity);
    }

     @Override
    public void setIntakeLiftSpeed(double speed) {
        intakeLiftPID.setSetpoint(speed, ControlType.kVelocity);
    }

    @Override
    public void setIntakeLiftPos(double pos) {
        intakeLiftPID.setSetpoint(pos, ControlType.kPosition);
    }

    @Override
    public void zeroIntakeLift() {
        intakeLift.getEncoder().setPosition(0.0);
    }

    @Override
    public double getIntakePosition() {
        return intakeLift.getEncoder().getPosition();
    }
}


