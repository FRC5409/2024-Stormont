package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.shuffleboard.ShuffleboardManager;

public class Indexer extends SubsystemBase {
    private static Indexer instance = null;

    // Motors
    private final CANSparkMax motor;

    // Sensors
    // private final DigitalInput sensor;

    // Shuffleboard
    private final ShuffleboardManager sb;

    private Indexer() {
        motor = initMotor();

        // sensor = new DigitalInput(IndexerConstants.IR_SENSOR_PORT);

        sb = new ShuffleboardManager("Indexer");
        sb.addEntry("Motor Speed", () -> motor.getEncoder().getVelocity());
        // sb.addEntry("Sensor Value", () -> checkIR());
    }

    private CANSparkMax initMotor() {
        CANSparkMax motor = new CANSparkMax(IndexerConstants.MOTOR_ID, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(IndexerConstants.CURRENT_LIMIT);

        motor.burnFlash();
        return motor;
    }

    // Get subsystem
    public static Indexer getInstance() {
        if (instance == null)
            instance = new Indexer();
        return instance;
    }

    /**
     * Set voltage of rollers.
     * 
     * @param speed Between -12 to 12.
     */
    public void setVoltage(double volts) {
        motor.set(volts);
    }

    /**
     * Get voltage of rollers.
     * 
     * @return The set voltage between -12 to 12.
     */
    public double getVoltage() {
        return motor.getBusVoltage();
    }

    /**
     * Gets the IR sensor value.
     * 
     * @return True if laser is interrupted.
     */
    // public boolean checkIR() {
    // return !sensor.get();
    // }

    /**
     * Gets the velocity of the indexer rollers in RPM.
     * 
     * @return RPM of motor.
     */
    public double getRPM() {
        return motor.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }
}
