package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class HopperIOSpark implements HopperIO {
    private final SparkFlex hopperMotor;
    private final SparkClosedLoopController hopperPID;
    private double setpoint;

    public HopperIOSpark() {
        hopperMotor = new SparkFlex(HopperConstants.hopperMotorID, MotorType.kBrushless);
        hopperPID = hopperMotor.getClosedLoopController();
        //0.008;
        FeedForwardConfig ffConf = new FeedForwardConfig();
        ffConf.kV(0.00245);


        setpoint = 0;

        SparkFlexConfig hopperConfig = new SparkFlexConfig();
        hopperConfig.idleMode(IdleMode.kBrake);
        hopperConfig.closedLoop.p(0.0006).i(0).d(0.00192);
        hopperConfig.closedLoop.apply(ffConf);

        hopperConfig.smartCurrentLimit(70);
        hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.hopperAppliedVolts = hopperMotor.getAppliedOutput();
        inputs.hopperConnected = true;
        inputs.hopperCurrentAmps = hopperMotor.getOutputCurrent();
        inputs.hopperPosition = Degrees.of(hopperMotor.getEncoder().getPosition());
        inputs.hopperVelocity = RotationsPerSecond.of(hopperMotor.getEncoder().getVelocity() / 60);
    }

    @Override
    public void setHopperSpeed(double speed) {
        setpoint = speed;
        hopperPID.setSetpoint(speed, ControlType.kVelocity);
        
    }

    @Override
    public double getSetpoint() {
        return setpoint;
    }
}