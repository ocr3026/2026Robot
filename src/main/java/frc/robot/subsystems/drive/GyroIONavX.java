/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Hertz;
import static frc.robot.Util.Offsets.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.hal.SerialPortJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Queue;

public class GyroIONavX implements GyroIO {
  private final AHRS navX =
      new AHRS(NavXComType.kI2C, (byte) DriveConstants.odometryFrequency.in(Hertz));

  private final AHRSData ahrs = new AHRSData();
  private boolean isConnected = false;

  private int portHandle;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {
    StringBuilder portName = new StringBuilder("/dev/");

    try {
      Process process = Runtime.getRuntime().exec("ls -l /dev/serial/by-id");
      StringBuilder output = new StringBuilder();

      try (BufferedReader reader =
          new BufferedReader(new InputStreamReader(process.getInputStream()))) {
        String line;
        while ((line = reader.readLine()) != null) {
          output.append(line).append("\n");
        }

        String str = output.toString();
        if (str.contains("Kauai_Labs")) {
          int indKuaui = str.indexOf("Kauai_Labs");
          String sub = str.substring(indKuaui);
          int indTTy = sub.indexOf("tty");
          String portStr = sub.substring(indTTy);
          portName.append(portStr);
        }
      }
    } catch (IOException e) {
      System.err.println("Failed to find input from command");
    }
    String realname = portName.toString().replaceAll("\\s+", "");
    System.out.println(realname);
    try {
      portHandle = SerialPortJNI.serialInitializePortDirect((byte) 3, realname);
      SerialPortJNI.serialSetBaudRate(portHandle, 115200);
      SerialPortJNI.serialSetDataBits(portHandle, (byte) 8);
      SerialPortJNI.serialSetParity(portHandle, (byte) 0);
      SerialPortJNI.serialSetStopBits(portHandle, (byte) 10);
      isConnected = true;
      System.out.println("Succeeded in initializing NavX Gyro at Serial port: " + realname + "!");
    } catch (Exception e) {
      System.out.println(
          "FATAL: Failed to initialize NavX Gyro with serial port: " + realname + "!");
      isConnected = false;
    }

    // navX.enableLogging(true);
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    
    // yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(navX::getYaw);
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(ahrs::getYaw);
    // System.out.println("NavxPort: " + navX.getPort());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // inputs.connected = navX.isConnected();
    // inputs.yawPosition = Rotation2d.fromDegrees(navX.getYaw());
    // inputs.yawVelocityRadPerSec = Units.degreesToRadians(navX.getRawGyroZ());
    // inputs.odometryYawTimestamps =
    //     yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    // inputs.odometryYawPositions = yawPositionQueue.stream()
    //     .map((Double value) -> Rotation2d.fromDegrees(-value))
    //     .toArray(Rotation2d[]::new);

    inputs.connected = isConnected;
    inputs.yawPosition = Rotation2d.fromDegrees(ahrs.getYaw());
    // inputs.yawVelocityRadPerSec = Units.degreesToRadians(ahrs.getRawGyroZ());
    inputs.yawVelocityRadPerSec = ahrs.getRawGyroZ();
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();

    inputs.odometryYawPositions = yawPositionQueue.stream()
        .map((Double value) -> Rotation2d.fromDegrees(-value))
        .toArray(Rotation2d[]::new);

    yawTimestampQueue.clear();
    yawPositionQueue.clear();

    int bytesRecieved = SerialPortJNI.serialGetBytesReceived(portHandle);

    byte[] buffer = new byte[bytesRecieved];
    int bytesRead = SerialPortJNI.serialRead(portHandle, buffer, buffer.length);
    if (buffer.length > 0) {
      isConnected = true;
      if (Integer.toHexString(buffer[0]).matches("21")
          && Integer.toHexString(buffer[1]).matches("23")
          && Integer.toHexString(buffer[3]).matches("70")) {
        ahrs.update(buffer);
      }
      // System.out.println("Parsed AHRS Yaw: " + ahrs.getYaw());
      // System.out.println("Parsed Pitch : " + ahrs.getPitch());
      // System.out.println("Parsed Roll: " + ahrs.getRoll());
    } else {
      isConnected = false;
    }
  }

  public class AHRSData {
    // Data Fields
    private float yaw, pitch, roll, compassHeading, altitude, fusedHeading;
    private float accelX, accelY, accelZ, velX, velY, velZ;
    private float dispX, dispY, dispZ;
    private float quatW, quatX, quatY, quatZ;
    private float temp;
    private byte opStatus, sensorStatus, calStatus, selfTestStatus;
    private float rawGyroZ;
    private float prevYaw = 0.0f;
    private long prevTimestamp = -1;
    private final ByteBuffer buffer = ByteBuffer.allocate(1024).order(ByteOrder.LITTLE_ENDIAN);

    /**
     * Updates all fields by parsing a raw byte array using the provided offsets
     * @param rawPacket The full byte array received from the serial port
     */
    public void update(byte[] rawPacket) {

      buffer.clear();

      if (rawPacket.length > buffer.capacity()) {
        System.err.println("Packet too large: " + rawPacket.length + " bytes");
        return;
      }
      buffer.put(rawPacket, 0, rawPacket.length);
      buffer.flip();

      // Signed Hundredths
      this.yaw = buffer.getShort(YAW_OFFSET) / 100.0f;
      long curTimestamp = System.currentTimeMillis();
      this.pitch = buffer.getShort(PITCH_OFFSET) / 100.0f;
      this.roll = buffer.getShort(ROLL_OFFSET) / 100.0f;
      this.temp = buffer.getShort(MPU_TEMP_OFFSET) / 100.0f;

      if (prevTimestamp != -1) {
        float deltaTime = (curTimestamp - prevTimestamp) / 1000.0f;

        if (deltaTime > 0) {
          float deltaYaw = yaw - prevYaw;

          if (deltaYaw > 180) deltaYaw -= 360;
          if (deltaYaw < -180) deltaYaw += 360;

          this.rawGyroZ = deltaYaw / deltaTime;
        }
      }

      this.prevYaw = this.yaw;
      this.prevTimestamp = curTimestamp;

      // Unsigned Hundredths (using 0xFFFF to treat short as unsigned)
      this.compassHeading = (buffer.getShort(COMPASS_HEADING_OFFSET) & 0xFFFF) / 100.0f;
      this.fusedHeading = (buffer.getShort(FUSED_HEADING_OFFSET) & 0xFFFF) / 100.0f;

      // Signed Thousandths
      this.accelX = buffer.getShort(LINEAR_ACCEL_X_OFFSET) / 1000.0f;
      this.accelY = buffer.getShort(LINEAR_ACCEL_Y_OFFSET) / 1000.0f;
      this.accelZ = buffer.getShort(LINEAR_ACCEL_Z_OFFSET) / 1000.0f;

      // Signed 16:16 Fixed Point
      this.altitude = buffer.getInt(ALTITUDE_OFFSET) / 65536.0f;
      this.velX = buffer.getInt(VELOCITY_X_OFFSET) / 65536.0f;
      this.velY = buffer.getInt(VELOCITY_Y_OFFSET) / 24.0f; // Simplified: 24 is just the offset
      this.velY = buffer.getInt(VELOCITY_Y_OFFSET) / 65536.0f;
      this.velZ = buffer.getInt(VELOCITY_Z_OFFSET) / 65536.0f;
      this.dispX = buffer.getInt(DISPLACEMENT_X_OFFSET) / 65536.0f;
      this.dispY = buffer.getInt(DISPLACEMENT_Y_OFFSET) / 65536.0f;
      this.dispZ = buffer.getInt(DISPLACEMENT_Z_OFFSET) / 65536.0f;

      // Signed Pi Radians (Quaternions)
      this.quatW = buffer.getShort(QUATERNION_W_OFFSET) / 16384.0f;
      this.quatX = buffer.getShort(QUATERNION_X_OFFSET) / 16384.0f;
      this.quatY = buffer.getShort(QUATERNION_Y_OFFSET) / 16384.0f;
      this.quatZ = buffer.getShort(QUATERNION_Z_OFFSET) / 16384.0f;

      // Status Bytes
      this.opStatus = buffer.get(OP_STATUS_OFFSET);
      this.sensorStatus = buffer.get(SENSOR_STATUS_OFFSET);
      this.calStatus = buffer.get(CAL_STATUS_OFFSET);
      this.selfTestStatus = buffer.get(SELFTEST_STATUS_OFFSET);

      // int msgLen = (int) buffer.get(2);
      // char messageId = (char) buffer.get(3);
      // System.out.println("Current Message ID: " + messageId);
      // System.out.println("Current Message Length: " + msgLen);
    }

    public float getYaw() {
      return yaw;
    }

    public float getPitch() {
      return pitch;
    }

    public float getRoll() {
      return roll;
    }

    public float getCompassHeading() {
      return compassHeading;
    }

    public float getAltitude() {
      return altitude;
    }

    public float getFusedHeading() {
      return fusedHeading;
    }

    public float getAccelX() {
      return accelX;
    }

    public float getAccelY() {
      return accelY;
    }

    public float getAccelZ() {
      return accelZ;
    }

    public float getVelX() {
      return velX;
    }

    public float getVelY() {
      return velY;
    }

    public float getVelZ() {
      return velZ;
    }

    public float getDispX() {
      return dispX;
    }

    public float getDispY() {
      return dispY;
    }

    public float getDispZ() {
      return dispZ;
    }

    public float getQuatW() {
      return quatW;
    }

    public float getQuatX() {
      return quatX;
    }

    public float getQuatY() {
      return quatY;
    }

    public float getQuatZ() {
      return quatZ;
    }

    public float getTemp() {
      return temp;
    }

    public byte getOpStatus() {
      return opStatus;
    }

    public byte getSensorStatus() {
      return sensorStatus;
    }

    public byte getCalStatus() {
      return calStatus;
    }

    public byte getSelfTestStatus() {
      return selfTestStatus;
    }

    public float getRawGyroZ() {
      return rawGyroZ;
    }

    public void setYaw(float yaw) {
      this.yaw = yaw;
    }
  }
}
