package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.shuffleboard.ShuffleboardManager;

public class Intake extends SubsystemBase {

	private static Intake instance = null;

	// Motors
	private final CANSparkMax rollersMotor;

	// Sensors
	private final DigitalInput irSensor;

	// Shuffleboard
	private final ShuffleboardManager sb;

	private Intake() {
		// Motors
		rollersMotor = new CANSparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);

		configMotor(rollersMotor, false);

		// Laser sensor
		irSensor = new DigitalInput(0);

		// Shuffleboard
		sb = new ShuffleboardManager("Intake");
		sb.addEntry("Motor Speed", () -> rollersMotor.getEncoder().getVelocity());
		sb.addEntry("Sensor Value", () -> irSensor.get());
	}

	// Get subsystem
	public static Intake getInstance() { if (instance == null) instance = new Intake(); return instance; }

	private void configMotor(CANSparkMax motor, boolean isInverted) {
		motor.restoreFactoryDefaults();
		motor.setInverted(isInverted);
		motor.setIdleMode(IdleMode.kBrake);
		motor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);

		motor.burnFlash();
	}

	/**
	 * Gets the IR sensor value.
	 * @return True if laser is interrupted.
	 */
	public boolean getSensorInterrupted() {
		return !irSensor.get();
	}

	/**
	 * Gets the velocity of the intake rollers in RPM.
	 * @return RPM of motor.
	 */
	public double getRPM() {
		return rollersMotor.getEncoder().getVelocity();
	}

	/**
	 * Set voltage of rollers.
	 * @param speed Between -12 to 12.
	 */
	public void setVoltage(double voltage) {
		rollersMotor.set(voltage);
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