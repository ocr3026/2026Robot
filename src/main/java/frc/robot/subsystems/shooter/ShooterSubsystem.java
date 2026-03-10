/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.shooter;

import com.orangefrc.annotation.GenerateJson;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  @GenerateJson
  public class Shooter {
    double sP;
    double sI;
    double sD;
    double sV;
    double sMaxAccel;

    double s2P;
    double s2I;
    double s2D;
    double s2V;
    double s2MaxAccel;

    double kickP;
    double kickI;
    double kickD;
    double kickV;
    double kickMaxAccel;
  }

  ShooterJson json = new ShooterJson();

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  public void runShooter(double speed) {
    io.setShooterSpeed(speed);
  }

  public void runShooter2(double speed) {
    io.setShooter2Speed(speed);
  }

  public void runShooterKickup(double speed) {
    io.setKickupSpeed(speed);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    json.updateVals();
    if (json.hasUpdated()) {
      io.updatePID(
          json.getsP(),
          json.getsI(),
          json.getsD(),
          json.getsV(),
          json.getsMaxAccel(),
          json.gets2P(),
          json.gets2I(),
          json.gets2D(),
          json.gets2V(),
          json.gets2MaxAccel(),
          json.getkickP(),
          json.getkickI(),
          json.getkickD(),
          json.getkickV(),
          json.getkickMaxAccel());
    }
  }

  @Override
  public void simulationPeriodic() {}
}
