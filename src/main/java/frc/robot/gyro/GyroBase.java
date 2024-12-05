package frc.robot.gyro;

import java.util.Date;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

public class GyroBase implements Gyro, Sendable, AutoCloseable {

    private static class AcquireTask implements Runnable {
        private final GyroBase imu;

        public AcquireTask(final GyroBase imu) {
            this.imu = imu;
        }

        @Override
        public void run() {
            imu.acquire();
        }
    }

    private Thread m_acquire_task;
  
    // * Static Constants */
    private static final double rad_to_deg = 57.2957795;
    private static final double deg_to_rad = 0.0174532;
    private static final double grav = 9.81;

    /* User-specified yaw axis */
    private GyroAxis m_yaw_axis = GyroAxis.kZ;

    /* Offset data storage */
    private double[] m_offset_data_gyro_rate_x;
    private double[] m_offset_data_gyro_rate_y;
    private double[] m_offset_data_gyro_rate_z;

    /* Instant raw output variables */
    private double m_gyro_rate_x = 0.0;
    private double m_gyro_rate_y = 0.0;
    private double m_gyro_rate_z = 0.0;
    private double m_accel_x = 0.0;
    private double m_accel_y = 0.0;
    private double m_accel_z = 0.0;

    /* IMU gyro offset variables */
    private double m_gyro_rate_offset_x = 0.0;
    private double m_gyro_rate_offset_y = 0.0;
    private double m_gyro_rate_offset_z = 0.0;
    private int m_avg_size = 0;
    private int m_accum_count = 0;

    /* Integrated gyro angle variables */
    private double m_integ_gyro_angle_x = 0.0;
    private double m_integ_gyro_angle_y = 0.0;
    private double m_integ_gyro_angle_z = 0.0;

    /* Complementary filter variables */
    private double m_dt = 0.0;
    private double m_alpha = 0.0;
    private double m_tau = 1.0;
    private double m_compAngleX = 0.0;
    private double m_compAngleY = 0.0;
    private double m_accelAngleX = 0.0;
    private double m_accelAngleY = 0.0;

    /* State variables */
    private volatile boolean m_thread_active = false;
    private volatile boolean m_first_run = true;
    private boolean m_start_up_mode = true;
    private boolean useFakeGyro = !RobotBase.isReal();

    private SimDevice m_simDevice;
    private SimDouble m_simGyroAngleX;
    private SimDouble m_simGyroAngleY;
    private SimDouble m_simGyroAngleZ;
    private SimDouble m_simGyroRateX;
    private SimDouble m_simGyroRateY;
    private SimDouble m_simGyroRateZ;
    private SimDouble m_simAccelX;
    private SimDouble m_simAccelY;
    private SimDouble m_simAccelZ;

    private DigitalOutput m_status_led;

    public GyroBase() {
        this(9);
    }

    public GyroBase(int readyPin) {
        m_acquire_task = new Thread(new AcquireTask(this));
        
        m_simDevice = SimDevice.create("Gyro", readyPin);
        if (m_simDevice != null) {
            m_simGyroAngleX = m_simDevice.createDouble("gyro_angle_x", SimDevice.Direction.kInput, 0.0);
            m_simGyroAngleY = m_simDevice.createDouble("gyro_angle_y", SimDevice.Direction.kInput, 0.0);
            m_simGyroAngleZ = m_simDevice.createDouble("gyro_angle_z", SimDevice.Direction.kInput, 0.0);
            m_simGyroRateX = m_simDevice.createDouble("gyro_rate_x", SimDevice.Direction.kInput, 0.0);
            m_simGyroRateY = m_simDevice.createDouble("gyro_rate_y", SimDevice.Direction.kInput, 0.0);
            m_simGyroRateZ = m_simDevice.createDouble("gyro_rate_z", SimDevice.Direction.kInput, 0.0);
            m_simAccelX = m_simDevice.createDouble("accel_x", SimDevice.Direction.kInput, 0.0);
            m_simAccelY = m_simDevice.createDouble("accel_y", SimDevice.Direction.kInput, 0.0);
            m_simAccelZ = m_simDevice.createDouble("accel_z", SimDevice.Direction.kInput, 0.0);
        }

        if (m_simDevice == null) {
            if (useFakeGyro) {
                DriverStation.reportWarning(
                    "useFakeGyro: Skipping initial calibration delay.", false);
            } else {
                // Notify DS that IMU calibration delay is active
                DriverStation.reportWarning(
                    "IMU Detected. Starting initial calibration delay.", false);
                // Wait for whatever time the user set as the start-up delay
                try {
                    Thread.sleep((long) (10 * 1000));
                } catch (InterruptedException e) {
                }
                // Execute calibration routine
                calibrate();
                // Reset accumulated offsets
                reset();
                // Indicate to the acquire loop that we're done starting up
                m_start_up_mode = false;
                // Let the user know the IMU was initiallized successfully
                DriverStation.reportWarning("IMU Successfully Initialized!", false);

                // Drive MXP PWM5 (IMU ready LED) low (active low)
                m_status_led = new DigitalOutput(readyPin);
                m_status_led.set(true);
            }
        }

        // Report usage and post data to DS
        HAL.report(tResourceType.kResourceType_Gyro, 0);
    }

    public void calibrate() {
        synchronized (this) {
            int gyroAverageSize = Math.min(m_accum_count, m_avg_size);
            double accum_gyro_rate_x = 0.0;
            double accum_gyro_rate_y = 0.0;
            double accum_gyro_rate_z = 0.0;
            for (int i = 0; i < gyroAverageSize; i++) {
              accum_gyro_rate_x += m_offset_data_gyro_rate_x[i];
              accum_gyro_rate_y += m_offset_data_gyro_rate_y[i];
              accum_gyro_rate_z += m_offset_data_gyro_rate_z[i];
            }
            m_gyro_rate_offset_x = accum_gyro_rate_x / gyroAverageSize;
            m_gyro_rate_offset_y = accum_gyro_rate_y / gyroAverageSize;
            m_gyro_rate_offset_z = accum_gyro_rate_z / gyroAverageSize;
            m_integ_gyro_angle_x = 0.0;
            m_integ_gyro_angle_y = 0.0;
            m_integ_gyro_angle_z = 0.0;
        }
    }

    public int setYawAxis(GyroAxis yaw_axis) {
        if (m_yaw_axis == yaw_axis) {
            return 1;
        }
        m_yaw_axis = yaw_axis;
        reset();
        return 0;
    }

    public static int toUShort(byte[] buf) {
        return buf[0] << 8 | buf[1];
    }

    // readRegister -  must override in child class
    public int readRegister(final int reg) {
        return 0;
    }
  
    // writeRegister -  must override in child class
    public void writeRegister(final int reg, final int val) {
    }
  
    public void reset() {
        synchronized (this) {
            m_integ_gyro_angle_x = 0.0;
            m_integ_gyro_angle_y = 0.0;
            m_integ_gyro_angle_z = 0.0;
        }      
    }

    // close -  must override in child class
    public void close() {
      if (m_thread_active) {
        m_thread_active = false;
        try {
          if (m_acquire_task != null) {
            m_acquire_task.join();
            m_acquire_task = null;
          }
        } catch (InterruptedException e) {
        }
      }
      if (m_simDevice != null) {
        m_simDevice.close();
        m_simDevice = null;
      }
      System.out.println("Finished cleaning up after the Gyro driver.");
    }

    // getSensorData -  must override in child class
    public double[] getSensorData() {
        double[] sensorData = new double[6];

        return sensorData;
    }
    
    public void acquire() {
        // Set up buffers and variables
        int bufferAvgIndex = 0;
        double previous_timestamp = 0.0;
        double gyro_rate_x_si = 0.0;
        double gyro_rate_y_si = 0.0;
        double accel_x_si = 0.0;
        double accel_y_si = 0.0;
        double accel_z_si = 0.0;
        double compAngleX = 0.0;
        double compAngleY = 0.0;
        double accelAngleX = 0.0;
        double accelAngleY = 0.0;

        while (true) {
            // Sleep loop for 5ms
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
            }

            if (m_thread_active) {
                double[] sensorData = getSensorData();

                m_dt = ((double) (new Date()).getTime() - previous_timestamp) / 1000000.0;

                // Convert scaled sensor data to SI units (for tilt calculations)
                // TODO: Should the unit outputs be selectable?
                gyro_rate_x_si = sensorData[0] * deg_to_rad;
                gyro_rate_y_si = sensorData[1] * deg_to_rad;
                // gyro_rate_z_si = gyro_rate_z * deg_to_rad;
                accel_x_si = sensorData[3] * grav;
                accel_y_si = sensorData[4] * grav;
                accel_z_si = sensorData[5] * grav;
                // Store timestamp for next iteration
                previous_timestamp = (new Date()).getTime();
                // Calculate alpha for use with the complementary filter
                m_alpha = m_tau / (m_tau + m_dt);

                // Calculate complementary filter
                if (m_first_run) {
                    // Set up inclinometer calculations for first run
                    accelAngleX =
                        Math.atan2(
                            -accel_x_si,
                            Math.sqrt((accel_y_si * accel_y_si) + (-accel_z_si * -accel_z_si)));
                    accelAngleY =
                        Math.atan2(
                            accel_y_si,
                            Math.sqrt((-accel_x_si * -accel_x_si) + (-accel_z_si * -accel_z_si)));
                    compAngleX = accelAngleX;
                    compAngleY = accelAngleY;
                } else {
                    // Run inclinometer calculations
                    accelAngleX =
                        Math.atan2(
                            -accel_x_si,
                            Math.sqrt((accel_y_si * accel_y_si) + (-accel_z_si * -accel_z_si)));
                    accelAngleY =
                        Math.atan2(
                            accel_y_si,
                            Math.sqrt((-accel_x_si * -accel_x_si) + (-accel_z_si * -accel_z_si)));
                    accelAngleX = formatAccelRange(accelAngleX, -accel_z_si);
                    accelAngleY = formatAccelRange(accelAngleY, -accel_z_si);
                    compAngleX = compFilterProcess(compAngleX, accelAngleX, -gyro_rate_y_si);
                    compAngleY = compFilterProcess(compAngleY, accelAngleY, -gyro_rate_x_si);
                }

                // Update global variables and state
                synchronized (this) {
                    // Ignore first, integrated sample
                    if (m_first_run) {
                        m_integ_gyro_angle_x = 0.0;
                        m_integ_gyro_angle_y = 0.0;
                        m_integ_gyro_angle_z = 0.0;
                    } else {
                        // Accumulate gyro for offset calibration
                        // Add to buffer
                        bufferAvgIndex = m_accum_count % m_avg_size;
                        m_offset_data_gyro_rate_x[bufferAvgIndex] = sensorData[0];
                        m_offset_data_gyro_rate_y[bufferAvgIndex] = sensorData[1];
                        m_offset_data_gyro_rate_z[bufferAvgIndex] = sensorData[2];
                        // Increment counter
                        m_accum_count++;
                    }
                    
                    if (!m_start_up_mode) {
                        m_gyro_rate_x = sensorData[0];
                        m_gyro_rate_y = sensorData[1];
                        m_gyro_rate_z = sensorData[2];
                        m_accel_x = sensorData[3];
                        m_accel_y = sensorData[4];
                        m_accel_z = sensorData[5];
                        m_compAngleX = compAngleX * rad_to_deg;
                        m_compAngleY = compAngleY * rad_to_deg;
                        m_accelAngleX = accelAngleX * rad_to_deg;
                        m_accelAngleY = accelAngleY * rad_to_deg;
                        // Accumulate gyro for angle integration and publish to global variables
                        m_integ_gyro_angle_x += (sensorData[0] - m_gyro_rate_offset_x) * m_dt;
                        m_integ_gyro_angle_y += (sensorData[1] - m_gyro_rate_offset_y) * m_dt;
                        m_integ_gyro_angle_z += (sensorData[2] - m_gyro_rate_offset_z) * m_dt;
                    }
                }
                m_first_run = false;
            } else {
                previous_timestamp = 0.0;
                gyro_rate_x_si = 0.0;
                gyro_rate_y_si = 0.0;
                // gyro_rate_z_si = 0.0;
                accel_x_si = 0.0;
                accel_y_si = 0.0;
                accel_z_si = 0.0;
                compAngleX = 0.0;
                compAngleY = 0.0;
                accelAngleX = 0.0;
                accelAngleY = 0.0;
            }
        }
    }

  /**
   * @param compAngle
   * @param accAngle
   * @return
   */
  private double formatFastConverge(double compAngle, double accAngle) {
    if (compAngle > accAngle + Math.PI) {
      compAngle = compAngle - 2.0 * Math.PI;
    } else if (accAngle > compAngle + Math.PI) {
      compAngle = compAngle + 2.0 * Math.PI;
    }
    return compAngle;
  }

  /**
   * @param compAngle
   * @return
   */
  private double formatRange0to2PI(double compAngle) {
    while (compAngle >= 2 * Math.PI) {
      compAngle = compAngle - 2.0 * Math.PI;
    }
    while (compAngle < 0.0) {
      compAngle = compAngle + 2.0 * Math.PI;
    }
    return compAngle;
  }

  /**
   * @param accelAngle
   * @param accelZ
   * @return
   */
  private double formatAccelRange(double accelAngle, double accelZ) {
    if (accelZ < 0.0) {
      accelAngle = Math.PI - accelAngle;
    } else if (accelZ > 0.0 && accelAngle < 0.0) {
      accelAngle = 2.0 * Math.PI + accelAngle;
    }
    return accelAngle;
  }

  /**
   * @param compAngle
   * @param accelAngle
   * @param omega
   * @return
   */
  private double compFilterProcess(double compAngle, double accelAngle, double omega) {
    compAngle = formatFastConverge(compAngle, accelAngle);
    compAngle = m_alpha * (compAngle + omega * m_dt) + (1.0 - m_alpha) * accelAngle;
    compAngle = formatRange0to2PI(compAngle);
    if (compAngle > Math.PI) {
      compAngle = compAngle - 2.0 * Math.PI;
    }
    return compAngle;
  }
  
    public double getAngle() {
        switch (m_yaw_axis) {
            case kX:
              return getGyroAngleX();
            case kY:
              return getGyroAngleY();
            case kZ:
              return getGyroAngleZ();
            default:
              return 0.0;
        }
    }
  
    public double getRate() {
        switch (m_yaw_axis) {
            case kX:
              return getGyroRateX();
            case kY:
              return getGyroRateY();
            case kZ:
              return getGyroRateZ();
            default:
              return 0.0;
        }      
    }
  
    public GyroAxis getYawAxis() {
        return m_yaw_axis;
    }
  
    public synchronized double getGyroAngleX() {
        if (m_simGyroAngleX != null) {
            return m_simGyroAngleX.get();
        }
        return m_integ_gyro_angle_x;              
    }
  
    public synchronized double getGyroAngleY() {
        if (m_simGyroAngleY != null) {
            return m_simGyroAngleY.get();
        }
        return m_integ_gyro_angle_y;        
    }
  
    public synchronized double getGyroAngleZ() {
        if (m_simGyroAngleZ != null) {
            return m_simGyroAngleZ.get();
        }
        return m_integ_gyro_angle_z;        
    }
  
    public synchronized double getGyroRateX() {
        if (m_simGyroRateX != null) {
            return m_simGyroRateX.get();
        }
        return m_gyro_rate_x;        
    }
  
    public synchronized double getGyroRateY() {
        if (m_simGyroRateY != null) {
            return m_simGyroRateY.get();
        }
        return m_gyro_rate_y;      
    }
  
    public synchronized double getGyroRateZ() {
        if (m_simGyroRateZ != null) {
            return m_simGyroRateZ.get();
        }
        return m_gyro_rate_z;      
    }
  
    public synchronized double getAccelX() {
        if (m_simAccelX != null) {
            return m_simAccelX.get();
        }
        return m_accel_x * 9.81;      
    }
  
    public synchronized double getAccelY() {
        if (m_simAccelY != null) {
            return m_simAccelY.get();
        }
        return m_accel_y * 9.81;
    }
  
    public double getAccelZ() {
        if (m_simAccelZ != null) {
            return m_simAccelZ.get();
        }
        return m_accel_z * 9.81;        
    }
  
    public double getXComplementaryAngle() {
        return m_compAngleX;        
    }
  
    public double getYComplementaryAngle() {
        return m_compAngleY;        
    }
  
    public double getXFilteredAccelAngle() {
        return m_accelAngleX;        
    }
  
    public double getYFilteredAccelAngle() {
        return m_accelAngleY;        
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", this::getAngle, null);
    }  
}
