/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.orangefrc.annotation.GenerateJson;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  ClimberJson json = new ClimberJson();
  int timesupdated = 0;

  @GenerateJson
  public class Climber {
    double p = 0;
    double i = 0;
    double d = 0;
    double v = 0;
  }

  public ClimberSubsystem(ClimberIO io) {
    json.init();
    this.io = io;
  }

  public void climberUp(double speed) {
    io.setClimberSpeed(speed);
  }

  public void climberDown(double speed) {
    io.setClimberSpeed(speed);
  }

  @Override
  public void periodic() {
    json.updateVals();
    if (json.hasUpdated()) {
      timesupdated++;
      io.updatePID(json.getp(), json.geti(), json.getd(), json.getv());
      NetworkTableInstance.getDefault()
          .getTable("Tuning")
          .getStringTopic("ClimberJson/Hasupdated")
          .publish()
          .set("Has updated the pid" + timesupdated);
    }
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    SmartDashboard.putNumber("ClimberSetpoint", io.getSetpoint());
    SmartDashboard.putNumber(
        "ClimberError", (io.getSetpoint() - (60 * inputs.climberVelocity.in(RotationsPerSecond))));
  }

  @Override
  public void simulationPeriodic() {}
}
