package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kCANID;

public class Cartridge extends SubsystemBase {

    private static Cartridge instance = null;
    // motor
    private final CANSparkMax m_motor;
    // Sensor
    private final DigitalInput irSwitch;

    // shuffleboard
    // private final ShuffleboardTab sb_cartridge;

    public Cartridge() {
        // initialize motor
        m_motor = new CANSparkMax(kCANID.CARTRIDGE_MOTOR_ID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setSmartCurrentLimit(Constants.kCartridge.CURRENT_LIMIT);

        // initialize irSwitch
        irSwitch = new DigitalInput(1);

        // shuffleboard
        // sb_cartridge = Shuffleboard.getTab("Cartridge");
        // sb_cartridge.addBoolean("irSwitch", () -> !irSwitch.get());

    }

    // Get subsystem
    public static Cartridge getInstance() {
        if (instance == null) instance = new Cartridge();

        return instance;
    }

    /**
     * Sets voltage for manual extension
     *
     * @param voltage
     */
    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    public boolean checkir() {
        return !irSwitch.get();
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
