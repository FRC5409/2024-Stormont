package frc.robot.Utils;

import java.util.TimerTask;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Improved functionality for CommandXboxController
 * 
 * Added method rumbleController(force, time);
 * Added method getRightJoystickAngle();
 * Added method getLeftJoystickAngle();
 * 
 * @author Alexander Szura
 */
public class Controller extends CommandXboxController {

    private static Controller instance[] = new Controller[DriverStation.kJoystickPorts];

    private Timer m_timer;
    private double m_rumbleTime = -1;

    private TimerTask loopTask;

    /**
     * Creates a new Controller object
     * @param port Controller port on driverstation (0 - 5)
     */
    private Controller(int port) {
        super(port);
        m_timer = new Timer();
        resetTimer();

        loopTask = new TimerTask() {

            @Override
            public void run() {
                if (m_timer.advanceIfElapsed(m_rumbleTime) && m_timer.get() != -1.0) {
                    resetTimer();
                    m_rumbleTime = -1;
        
                    stopRumble();
                }
            }

        };

        new java.util.Timer().schedule(loopTask, 0, 50);
        
    }

    /**
     * Gets controller at PORT
     * @param port Controller port: 0 - 5
     * @return Controller class
     */
    public static Controller getPort(int port) {
        try {
            if (instance[port] == null) instance[port] = new Controller(port);
        } catch (Exception e) {
            throw new NullPointerException("Cannot get port: " + port + " because port: " + port + " does not exist");
        }

        return instance[port];
    }

    /**
     * Resets and stops all timer related activitys
     */
    private void resetTimer() {
        m_timer.stop();
        m_timer.reset();
        m_rumbleTime = -1;
    }

    /**
     * Rumble the controller at FORCE for TIME
     * @param force 0 to 1
     * @param time duration
     */
    public void rumbleController(double force, double time) {
        m_timer.start();
        m_rumbleTime = time;

        this.getHID().setRumble(RumbleType.kBothRumble, force);
    }

    /**
     * Rumble the controller at FORCE for TIME
     * @param force 0 to 1
     * @param time duration
     * @param type The type of rumble (kLeftRumble, kRightRumble, kBothRumble)
     */
    public void rumbleController(double force, double time, RumbleType type) {
        m_timer.start();
        m_rumbleTime = time;

        this.getHID().setRumble(type, force);
    }

    /**
     * @param deadBand the applied deadband in which the joystick will return the value -1.0
     * @return the given angle currently appied to the left joystick
     */
    public double getLeftJoystickAngle(double deadBand) {
        double x = this.getLeftX();
        double y = this.getLeftY();

        if (distanceCalculation(x, y) <= deadBand) return -1.0;

        return angleCalculation(x, y);
    }

    /**
     * @param deadBand the applied deadband in which the joystick will return the value -1.0
     * @return the given angle currently appied to the right joystick
     */
    public double getRightJoystickAngle(double deadBand) {
        double x = this.getRightX();
        double y = this.getRightY();

        if (distanceCalculation(x, y) <= deadBand) return -1.0;

        return angleCalculation(x, y);
    }

    private double angleCalculation(double x, double y) {
        return Math.toDegrees(Math.atan2(y, x));
    }

    private double distanceCalculation(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Stops rumbling the controller
     */
    public void stopRumble() {
        this.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

}
