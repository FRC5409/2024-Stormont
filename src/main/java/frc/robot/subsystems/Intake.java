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
import frc.robot.Constants.kCANID;
import frc.robot.Constants.kIntake;

public class Intake extends SubsystemBase {
	private static Intake instance = null;

	// Motors
	private final CANSparkMax motor;

	// PID
	private final SparkPIDController controller;

	// Sensors
	// Temporarily commented-out until IR sensor is attached
	// private final DigitalInput irSensor;

	// Shuffleboard
	private final ShuffleboardTab sb_tab;

	private Intake() {
		// Motors
		motor = new CANSparkMax(kCANID.INTAKE_MOTOR_ID, MotorType.kBrushless);
		configMotor(motor, false);

		// PID
		controller = motor.getPIDController();
		controller.setP(kIntake.KP);
		controller.setI(kIntake.KI);
		controller.setD(kIntake.KD);
		controller.setFF(kIntake.KFF);

		motor.burnFlash();

		// Laser sensor
		// Temporarily commented-out until IR sensor is attached
		// irSensor = new DigitalInput(0);

		// Shuffleboard
		sb_tab = Shuffleboard.getTab("Intake");
		sb_tab.addDouble("RPM", () -> getRPM());
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
	}

	/**
	 * Gets the IR sensor value.
	 * 
	 * @return True if laser is interrupted.
	 */
	// Temporarily commented-out until IR sensor is attached
	/*
	 * public boolean checkIR() {
	 * return !irSensor.get();
	 * }
	 */

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
	 * Set desired RPM of motor. Will set the setpoint of the PID controller.
	 * 
	 * @param rpm Desired RPM of motor.
	 */
	public void setRPM(double rpm) {
		controller.setReference(rpm, ControlType.kVelocity);
	}
}