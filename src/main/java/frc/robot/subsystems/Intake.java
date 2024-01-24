package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDrivetrain;
import frc.robot.shuffleboard.ShuffleboardManager;

public class Intake extends SubsystemBase {

	private static Intake instance = null;

	// Motors
	private final CANSparkMax outerRoller;
	private final CANSparkMax innerRoller;

	// Proximity sensor
	private final DigitalInput laserSensor;

	// Shuffleboard
	private final ShuffleboardManager sb;

	private Intake() {
		// Motors
		outerRoller = new CANSparkMax(0, MotorType.kBrushed);
		innerRoller = new CANSparkMax(0, MotorType.kBrushed);

		configMotor(outerRoller, false);
		configMotor(innerRoller, false);

		// Laser sensor
		laserSensor = new DigitalInput(0);

		// Shuffleboard
		sb = new ShuffleboardManager("Intake");
		sb.addEntry("Outer Roller Speed", () -> outerRoller.getEncoder().getVelocity());
		sb.addEntry("Inner Roller Speed", () -> innerRoller.getEncoder().getVelocity());
	}

	// Get subsystem
	public static Intake getInstance() {
		if (instance == null) instance = new Intake();

		return instance;
	}

	private void configMotor(CANSparkMax motor, boolean isInverted) {
		motor.restoreFactoryDefaults();
		motor.setInverted(isInverted);
		motor.setIdleMode(IdleMode.kBrake);
		motor.setSmartCurrentLimit(kDrivetrain.kCurrentLimit);
		motor.setClosedLoopRampRate(kDrivetrain.kClosedLoopRampRate);

		motor.burnFlash();
	}

	public boolean getSensorValue() {
		return laserSensor.get();
	}

	public void setOuterRoller(double output, boolean isVoltage) {
		if (isVoltage) outerRoller.setVoltage(output);
		else outerRoller.set(output);
	}

	public void setInnerRoller(double output, boolean isVoltage) {
		if (isVoltage) innerRoller.setVoltage(output);
		else innerRoller.set(output);
	}

	public void stopAllMotors() {
		outerRoller.setVoltage(0);
		innerRoller.setVoltage(0);
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