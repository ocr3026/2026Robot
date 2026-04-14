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
  /**
   * This is a freaky class that uses WPILibs SerialPort Java Native Interface to get raw data from the gyro on a serial port
   * The serial port the gyro is on is found by shelling into the RoboRIO, and running the command "ls -l /dev/serial/by-id"
   * There is a method that seraches for it on the output from that command in the constructor of this class.
   */
  private final AHRS navX =
      new AHRS(NavXComType.kI2C, (byte) DriveConstants.odometryFrequency.in(Hertz));

  private final AHRSData ahrs = new AHRSData();
  private boolean isConnected = false;

  private int portHandle;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {

    /**
     * Automatically search and initialize the port by using the command
     */
    // String builder to be able to append to it
    StringBuilder portName = new StringBuilder("/dev/");

    try {
      // Run the command in the runtime environment (the rio)
      Process process = Runtime.getRuntime().exec("ls -l /dev/serial/by-id");
      StringBuilder output = new StringBuilder();

      // Read the output from the command
      try (BufferedReader reader =
          new BufferedReader(new InputStreamReader(process.getInputStream()))) {
        String line;
        while ((line = reader.readLine()) != null) {
          // Add to the output stringbuilder
          output.append(line).append("\n");
        }

        String str = output.toString(); // Convert the output to a string
        if (str.contains(
            "Kauai_Labs")) { // Check to see if it contains this substring, which is the creator of
          // the NavX
          int indKuaui = str.indexOf("Kauai_Labs"); // Find the index if it does contain this
          String sub = str.substring(
              indKuaui); // Make a substring of the output, starting at where Kauai_Labs is
          // mentioned
          int indTTy =
              sub.indexOf("tty"); // Finds the first instance of "tty", which is what port it is on
          String portStr = sub.substring(
              indTTy,
              indTTy
                  + 4); // makes a substring containing just the "tty" + whatever number port it is
          // on
          portName.append(portStr); // Add the port name (ex. "tty0") to the portName
        }
      }
    } catch (IOException e) {
      System.err.println("Failed to find input from command");
    }
    String realname = portName.toString().replaceAll("\\s+", ""); // get rid of whitespace
    System.out.println(
        realname); // Make sure the name we are getting makes sense, just for the reader
    try {
      portHandle = SerialPortJNI.serialInitializePortDirect(
          (byte) 3, realname); // Intialize the port on port 3 with the device port name we derived
      SerialPortJNI.serialSetBaudRate(
          portHandle,
          115200); // Setup, setting baud Rate (google what that is) to whatever it says on
      // documentation for the NavX2
      SerialPortJNI.serialSetDataBits(portHandle, (byte)
          8); // define how many bits it takes to define one set of data - 8bits is default, 8 bits
      // in one byte = one char
      SerialPortJNI.serialSetParity(
          portHandle,
          (byte) 0); // Parity is error checking, the NavX does not do this so set it to 0
      SerialPortJNI.serialSetStopBits(
          portHandle,
          (byte) 10); // how many bits there are to indicate the end of a packet - 10 is one for
      // somereason, set to 10 for navX
      isConnected =
          true; // Make sure we can see if we have successfully connected to this serial port
      // through our jank method
      System.out.println("Succeeded in initializing NavX Gyro at Serial port: " + realname + "!");
    } catch (Exception e) {
      System.out.println(
          "FATAL: Failed to initialize NavX Gyro with serial port: " + realname + "!");
      isConnected = false; // Tell us if we havent successfully initialized our gyro
    }

    // navX.enableLogging(true);
    yawTimestampQueue =
        PhoenixOdometryThread.getInstance().makeTimestampQueue(); // Initialize timestamp queue

    // yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(navX::getYaw);
    yawPositionQueue = PhoenixOdometryThread.getInstance()
        .registerSignal(ahrs::getYaw); // Initialize yaw queue with the AHRS navX
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
    // inputs.yawVelocityRadPerSec = Units.degreesToRadians(ahrs.getRawGyroZ());
    inputs.yawVelocityRadPerSec = ahrs.getRawGyroZ();
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();

    inputs.odometryYawPositions = yawPositionQueue.stream()
        .map((Double value) -> Rotation2d.fromDegrees(-value))
        .toArray(Rotation2d[]::new);

    yawTimestampQueue.clear();
    yawPositionQueue.clear();

    int bytesRecieved = SerialPortJNI.serialGetBytesReceived(
        portHandle); // the int bytesRecieved shows the size of the packet we have gotten from the
    // gyro

    byte[] buffer = new byte
        [bytesRecieved]; // Make a new buffer array to hold all the data we recieved, making it the
    // size of the packet we recieved
    int bytesRead = SerialPortJNI.serialRead(
        portHandle,
        buffer,
        buffer
            .length); // Read the data from the port, putting it into the buffer array we previously
    // made - the return value just contains the amount of bytes read, we dont use
    // that for anything
    if (buffer.length > 0) {
      isConnected =
          true; // If we have actually recieved a message from the gyro, make sure we still think
      // it's connected
      if (Integer.toHexString(buffer[0]).matches("21")
          && Integer.toHexString(buffer[1]).matches("23")
          && Integer.toHexString(buffer[3])
              .matches(
                  "70")) { // This if statement makes sure it is an information update packet we get
        // from the navX, found this on their documentation/user guide
        ahrs.update(buffer); // call the function to update all our values
      }
    } else {
      isConnected =
          false; // If we haven't recieved a mmessage packet from the navX, say that we have
      // disconnected from it.
    }
  }

  /**
   * This class is a container for each instance of a navX through the USB port in the rio
   * Call the update function and pass in the right message packet for this gyro.
   */
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

    /*
     * A byte buffer is a container for our packets of data to more easily read the data we get.
     * We allocate a max capacity of 1024 bytes, with the order of Little Endian, meaning the most significant byte (the "first" byte)
     * is stored at the lowest memory address
     * Some serial ports use Big Endian, but the navX uses Little Endian
     */
    private final ByteBuffer buffer = ByteBuffer.allocate(1024).order(ByteOrder.LITTLE_ENDIAN);

    /**
     * Updates all fields by parsing a raw byte array using the provided offsets
     * @param rawPacket The full byte array received from the serial port
     */
    public void update(byte[] rawPacket) {

      /*
       * Make sure we clear the bytebuffer each time so we don't get "stale" data, while also not using up too much memory by creating a new one each period
       */
      buffer.clear();

      /*
       * Make sure our packet is the right length/ is the right message
       */
      if (rawPacket.length > buffer.capacity()) {
        System.err.println("Packet too large: " + rawPacket.length + " bytes");
        return;
      }
      buffer.put(rawPacket, 0, rawPacket.length); // Put our message packet in the buffer
      /*
       * Because items are put into the buffer at the beginning, the first value becomes the last
       * We flip the buffer to maintain first item at first spot
       */
      buffer.flip(); 

      /*
       *  We divide by 100 because the original value is in Signed Hundredths, meaning
       *  it is the original value * 100.0 rounded to nearest int
       */
      this.yaw = buffer.getShort(YAW_OFFSET) / 100.0f;
      // Get current timestamp so we can calculate the yaw velocity
      long curTimestamp = System.currentTimeMillis();
      this.pitch = buffer.getShort(PITCH_OFFSET) / 100.0f;
      this.roll = buffer.getShort(ROLL_OFFSET) / 100.0f;
      this.temp = buffer.getShort(MPU_TEMP_OFFSET) / 100.0f;

      if (prevTimestamp != -1) {
        float deltaTime = (curTimestamp - prevTimestamp)
            / 1000.0f; // divide by 1000 because system time is in miliseconds

        if (deltaTime > 0) {
          float deltaYaw = yaw - prevYaw;

          if (deltaYaw > 180) deltaYaw -= 360;
          if (deltaYaw < -180) deltaYaw += 360;

          this.rawGyroZ = deltaYaw / deltaTime;
        }
      }

      this.prevYaw = this.yaw;
      this.prevTimestamp = curTimestamp;

      /*
       * We use the "sign bit" (0xFFFF) to make sure all values we get are positive, unsigned hundredths 
       * We divide by 100 because the value is in hundredths (original value * 100) 
       */
      this.compassHeading = (buffer.getShort(COMPASS_HEADING_OFFSET) & 0xFFFF) / 100.0f;
      this.fusedHeading = (buffer.getShort(FUSED_HEADING_OFFSET) & 0xFFFF) / 100.0f;

      /*
       * Unsigned thousandths, we divide by 1000 to get original value (the packet value is origin value / 1000)
       */
      this.accelX = buffer.getShort(LINEAR_ACCEL_X_OFFSET) / 1000.0f;
      this.accelY = buffer.getShort(LINEAR_ACCEL_Y_OFFSET) / 1000.0f;
      this.accelZ = buffer.getShort(LINEAR_ACCEL_Z_OFFSET) / 1000.0f;

      /*
       * Signed 16:16, which is in Q notation (Q16:16) - look up Q notation wikipedia for more info
       * We get the raw value as an int, then divide by 2^16  (65536.0) in order to get the floating point value
       */
      this.altitude = buffer.getInt(ALTITUDE_OFFSET) / 65536.0f;
      this.velX = buffer.getInt(VELOCITY_X_OFFSET) / 65536.0f;
      this.velY = buffer.getInt(VELOCITY_Y_OFFSET) / 24.0f;
      this.velY = buffer.getInt(VELOCITY_Y_OFFSET) / 65536.0f;
      this.velZ = buffer.getInt(VELOCITY_Z_OFFSET) / 65536.0f;
      this.dispX = buffer.getInt(DISPLACEMENT_X_OFFSET) / 65536.0f;
      this.dispY = buffer.getInt(DISPLACEMENT_Y_OFFSET) / 65536.0f;
      this.dispZ = buffer.getInt(DISPLACEMENT_Z_OFFSET) / 65536.0f;

      /* 
       * These values are in signed pi radians (original value * 16384), so we divide by 16384 to get original value
       */
      this.quatW = buffer.getShort(QUATERNION_W_OFFSET) / 16384.0f;
      this.quatX = buffer.getShort(QUATERNION_X_OFFSET) / 16384.0f;
      this.quatY = buffer.getShort(QUATERNION_Y_OFFSET) / 16384.0f;
      this.quatZ = buffer.getShort(QUATERNION_Z_OFFSET) / 16384.0f;

      /*
       * Status values from the rest of the message
       */
      this.opStatus = buffer.get(OP_STATUS_OFFSET);
      this.sensorStatus = buffer.get(SENSOR_STATUS_OFFSET);
      this.calStatus = buffer.get(CAL_STATUS_OFFSET);
      this.selfTestStatus = buffer.get(SELFTEST_STATUS_OFFSET);
    }

    /*
     * A bunch of get methods so we can get all the data from this object
     */
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

    /*
     * We need this function to zero the gyro
     */
    public void setYaw(float yaw) {
      this.yaw = yaw;
    }
  }
}
