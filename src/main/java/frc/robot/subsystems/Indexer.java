package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCANID;
import frc.robot.Constants.kIndexer;

public class Indexer extends SubsystemBase {

	private static Indexer instance = null;

	// Motors
	private final CANSparkMax motor;

	// Sensors
	private final DigitalInput irSensor;

	// Shuffleboard
	// private final ShuffleboardTab sb_tab;

	private Indexer() {
		// Motors
		motor = new CANSparkMax(kCANID.INDEXER_MOTOR_ID, MotorType.kBrushless);

		configMotor(motor, false);

		// Laser sensor
		irSensor = new DigitalInput(kIndexer.IR_SENSOR_PORT);

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
		motor.setSmartCurrentLimit(kIndexer.CURRENT_LIMIT);

		motor.burnFlash();
	}

	/**
	 * Gets the IR sensor value.
	 *
	 * @return True if laser is interrupted.
	 */
	public boolean checkIR() {
		return !irSensor.get();
	}

	/**
	 * Set voltage of motor.
	 *
	 * @param volts
	 *            Between -12 to 12.
	 */
	public void setVoltage(double volts) {
		motor.setVoltage(volts);
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
