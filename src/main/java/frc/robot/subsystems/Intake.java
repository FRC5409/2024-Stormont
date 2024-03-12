package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCANID;
import frc.robot.Constants.kIntake;

public class Intake extends SubsystemBase {

	private static Intake instance = null;

	// Motors
	private final CANSparkMax motor;

	// Sensors
	// private final DigitalInput irSensor;

	private Intake() {
		// Motors
		motor = new CANSparkMax(kCANID.INTAKE_MOTOR_ID, MotorType.kBrushless);

		configMotor(motor, false);
	}

	// Get subsystem
	public static Intake getInstance() {
		if (instance == null)
			instance = new Intake();
		return instance;
	}

	private void configMotor(CANSparkMax motor, boolean isInverted) {
		motor.restoreFactoryDefaults();
		motor.setInverted(isInverted);
		motor.setIdleMode(IdleMode.kBrake);
		motor.setSmartCurrentLimit(kIntake.CURRENT_LIMIT);

		motor.burnFlash();
	}

	/**
	 * Set voltage of motor.
	 * 
	 * @param volts Between -12 to 12.
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