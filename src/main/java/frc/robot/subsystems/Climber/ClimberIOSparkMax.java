package frc.robot.subsystems.Climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.kClimber.kSpark;

public class ClimberIOSparkMax implements ClimberIO {

    private final CANSparkMax mot;
    private final RelativeEncoder encoder;
    private final SparkPIDController controller;

    private final ShuffleboardTab sb_spark;
    private final GenericEntry sb_kp;
    private final GenericEntry sb_ki;
    private final GenericEntry sb_kd;
    private final GenericEntry sb_kff;
    
    public ClimberIOSparkMax(int ID) {
        mot = new CANSparkMax(ID, MotorType.kBrushless);

        mot.restoreFactoryDefaults();
        mot.setSmartCurrentLimit(kSpark.CURRENT_LIMIT);
        mot.setInverted(kSpark.INVERTED);
        mot.setIdleMode(IdleMode.kBrake);

        encoder = mot.getEncoder();

        encoder.setPositionConversionFactor(kSpark.CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(kSpark.CONVERSION_FACTOR / 50.0);

        controller = mot.getPIDController();

        // TODO: Uncomment when PID tuning is complete

        // controller.setP(kSpark.kP);
        // controller.setI(kSpark.kI);
        // controller.setD(kSpark.kD);
        // controller.setFF(kSpark.kFF);

        // mot.burnFlash();

        sb_spark = Shuffleboard.getTab("Spark-Climber");

        sb_spark.addDouble("Position", () -> encoder.getPosition());

        sb_kp = sb_spark.add("kP", kSpark.kP).getEntry();
        sb_ki = sb_spark.add("kI", kSpark.kI).getEntry();
        sb_kd = sb_spark.add("kD", kSpark.kD).getEntry();
        sb_kff = sb_spark.add("kFF", kSpark.kFF).getEntry();
    }

    @Override
    public void debugPID() {
        controller.setP(sb_kp.getDouble(0.0));
        controller.setI(sb_ki.getDouble(0.0));
        controller.setD(sb_kd.getDouble(0.0));
        controller.setFF(sb_kff.getDouble(0.0));
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.motorConnected = true;
        inputs.motorVoltage = mot.get() * RobotController.getBatteryVoltage();
        inputs.motorCurrent = mot.getOutputCurrent();
        inputs.climberPosition = encoder.getPosition();
        inputs.motorTemp = mot.getMotorTemperature();
    }

    @Override
    public void setPosition(double position) {
        controller.setReference(position, ControlType.kPosition);
    }

    @Override
    public void stop() {
        mot.stopMotor();
    }

    @Override
    public void setVoltage(double voltage) {
        mot.setVoltage(voltage);
    }

    @Override
    public void zero() {
        encoder.setPosition(0.0);
    }

    @Override
    public String getIOName() {
        return "SparkMax";
    }
}
