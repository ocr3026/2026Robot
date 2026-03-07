package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.subsystems.hopper.HopperIO.HopperIOInputs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ClimberIOTalon implements ClimberIO {
    private final  TalonFX climberMotor;
    private double setpoint;

    public ClimberIOTalon() {
        climberMotor = new TalonFX(32);
        //0.008;
        FeedForwardConfig ffConf = new FeedForwardConfig();
        ffConf.kV(0.00245);


        setpoint = 0;

        // TalonFXConfiguration ClimberConfig = new TalonFXConfiguration();
        // var Slot0Configs = new Slot0Configs();
        // Slot0Configs = ClimberConfig.Slot0;
        // Slot0Configs.kP = 0.0;
        // Slot0Configs.kI = 0.0;
        // Slot0Configs.kD = 0.0;

        // climberMotor.getConfigurator().apply(ClimberConfig);
        zeroClimber();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberAppliedVolts =climberMotor.getSupplyVoltage().getValueAsDouble();
        inputs.climberConnected = true;
        inputs.climberCurrentAmps = climberMotor.getSupplyCurrent().getValueAsDouble();
        inputs.climberPosition = Rotations.of(climberMotor.getPosition().getValueAsDouble());
        inputs.climberVelocity = RotationsPerSecond.of(climberMotor.getVelocity().getValueAsDouble() / 60);
    }

    @Override
    public void setClimberSpeed(double speed) {
        setpoint = speed;
        climberMotor.setControl(new PositionDutyCycle(setpoint));
        //climberMotor.setControl(new DutyCycleOut(setpoint));
        
    }

        @Override
    public void setClimberPos(double pos) {
        climberMotor.setControl(new PositionDutyCycle(0));
        //Think rotation is 90?
    }

    @Override
    public double getSetpoint() {
        return setpoint;
    }
}