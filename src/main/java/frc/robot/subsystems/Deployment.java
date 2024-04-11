// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCANID;
import frc.robot.Constants.kDeployment;

public class Deployment extends SubsystemBase {
	// motor
	private final CANSparkMax m_motor;
	// controller
	private final SparkPIDController m_controller;
	// encoder
	private final RelativeEncoder s_encoder;

	private static Deployment instance = null;

	/**
	 * Creates a new Deployment. One CANSparkMax
	 */
	public Deployment() {
		// initialize motor
		m_motor = new CANSparkMax(kCANID.DEPLOYMENT_MOTOR_ID, MotorType.kBrushless);
		m_motor.restoreFactoryDefaults();
		m_motor.setIdleMode(IdleMode.kBrake);
		m_motor.setSmartCurrentLimit(kDeployment.CURRENT_LIMIT);

		// initialize controller
		m_controller = m_motor.getPIDController();
		configPID();

		// initialize encoder
		s_encoder = m_motor.getEncoder();
		zeroEncoder();

		m_motor.burnFlash();
	}

	public static Deployment getInstance() {
		if (instance == null)
			instance = new Deployment();

		return instance;
	}

	/**
	 * Sets PID values for motor controller
	 */
	public void configPID() {
		m_controller.setP(kDeployment.kPID.kSlowSlot.kP, kDeployment.kPID.kSlowSlot.slot);
		m_controller.setI(kDeployment.kPID.kSlowSlot.kI, kDeployment.kPID.kSlowSlot.slot);
		m_controller.setD(kDeployment.kPID.kSlowSlot.kD, kDeployment.kPID.kSlowSlot.slot);

		m_controller.setP(kDeployment.kPID.kFastSlot.kP, kDeployment.kPID.kFastSlot.slot);
		m_controller.setI(kDeployment.kPID.kFastSlot.kI, kDeployment.kPID.kFastSlot.slot);
		m_controller.setD(kDeployment.kPID.kFastSlot.kD, kDeployment.kPID.kFastSlot.slot);
	}

	public double getPosition() {
		return s_encoder.getPosition();
	}

	public void stopMot() {
		m_motor.stopMotor();
	}

	/**
	 * Zero encoder with encoder
	 *
	 * @param position
	 */
	public void zeroEncoder() {
		s_encoder.setPosition(0);
	}

	public boolean atSetpoint(double setpoint) {
		return MathUtil.isNear(setpoint, getPosition(), kDeployment.TOLERANCE);
	}

	/**
	 * Motor go to setpoint from constants
	 *
	 * @param setpoint
	 *            value
	 * @param outputVolts
	 *            value 0-12
	 */
	public void setPosition(double setpoint, int slot) {
		m_controller.setReference(setpoint, ControlType.kPosition, slot);
	}

	/**
	 * Sets voltage for manual extension
	 *
	 * @param voltage
	 */
	public void setVoltage(double voltage) {
		m_motor.setVoltage(voltage);
	}

	@Override
	public void periodic() {
	}
}
