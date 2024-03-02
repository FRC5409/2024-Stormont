package frc.robot.subsystems;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.kCartridge;

import com.ctre.phoenix6.signals.ControlModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Cartridge extends SubsystemBase {

    private static Cartridge instance = null;
    // motor
    private final CANSparkMax m_motor;
    // Sensor
    // private final DigitalInput irSwitch;
    // shuffleboard
    // private final ShuffleboardTab sb_cartridge;

    public Cartridge() {
        // initialize motor
        m_motor = new CANSparkMax(kCartridge.id_motor, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setSmartCurrentLimit(Constants.kCartridge.currentLimit);

        // initialize irSwitch
        // irSwitch = new DigitalInput(0);

        // shuffleboard
        // sb_cartridge = Shuffleboard.getTab("Cartridge");
        // sb_cartridge.addBoolean("irSwitch", () -> !irSwitch.get());

    }

    // Get subsystem
    public static Cartridge getInstance() {
        if (instance == null)
            instance = new Cartridge();

        return instance;
    }

    /**
     * Sets voltage for manual extension
     * 
     * @param voltage
     */

    public void roll(double voltage) {
        m_motor.setVoltage(voltage);
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