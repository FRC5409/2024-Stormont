package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {

    private static Indexer instance = null;

    // Motors
    private final CANSparkMax motor;

    // PID
    private final SparkPIDController controller;

    // Sensors
    // private final DigitalInput irSensor;

    // Shuffleboard
    // private final ShuffleboardTab sb_tab;

    private Indexer() {
        // Motors
        motor = new CANSparkMax(IndexerConstants.MOTOR_ID, MotorType.kBrushless);
        configMotor(motor, false);

        // PID
        controller = motor.getPIDController();
        controller.setP(IndexerConstants.KP);
        controller.setI(IndexerConstants.KI);
        controller.setD(IndexerConstants.KD);

        // Laser sensor
        // irSensor = new DigitalInput(0);

        // Shuffleboard
        // sb_tab = Shuffleboard.getTab("Indexer");
        // sb_tab.addBoolean("IR Sensor Value", () -> checkIR());
    }

    // Get subsystem
    public static Indexer getInstance() {
        if (instance == null)
            instance = new Indexer();
        return instance;
    }

    private void configMotor(CANSparkMax motor, boolean isInverted) {
        motor.restoreFactoryDefaults();
        motor.setInverted(isInverted);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(IndexerConstants.CURRENT_LIMIT);

        motor.burnFlash();
    }

    /**
     * Gets the IR sensor value.
     * 
     * @return True if laser is interrupted.
     */
    // public boolean checkIR() {
    // return !irSensor.get();
    // }

    /**
     * Gets the velocity of motor in RPM.
     * 
     * @return RPM of motor.
     */
    public double getRPM() {
        return motor.getEncoder().getVelocity();
    }

    /**
     * Manually set voltage of motor.
     * 
     * @param volts Between -12 and 12.
     */
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    /**
     * Get voltage of motor.
     * 
     * @return The set voltage between -12 and 12.
     */
    public double getVoltage() {
        return motor.getBusVoltage();
    }

    /**
     * Set desired RPM of motor. Will set the setpoint of the PID controller.
     * 
     * @param rpm Desired RPM of motor.
     */
    public void setRPM(double rpm) {
        controller.setReference(rpm, ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }

}