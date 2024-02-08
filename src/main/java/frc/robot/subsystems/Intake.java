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

	// Proximity sensor
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
		sb.addEntry("Speed of Rollers", () -> rollersMotor.getEncoder().getVelocity());
		sb.addEntry("Is Note Fed", () -> getSensorValue());
	}

	// Get subsystem
	public static Intake getInstance() { if (instance == null) instance = new Intake(); return instance; }

	private void configMotor(CANSparkMax motor, boolean isInverted) {
		motor.restoreFactoryDefaults();
		motor.setInverted(isInverted);
		motor.setIdleMode(IdleMode.kBrake);
		motor.setSmartCurrentLimit(IntakeConstants.MOTOR_CURRENT_LIMIT);
		motor.setClosedLoopRampRate(IntakeConstants.MOTOR_RAMP_RATE);

		motor.burnFlash();
	}

	public boolean getSensorValue() {
		return irSensor.get();
	}

	public void setSpeed(double speed) {
		rollersMotor.set(speed);
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