package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

/**
 * Subsystem for the claw that is positioned on the end of the pendulum.
 * As of now, simply controls a singular motor.
 */
public class Claw extends SubsystemBase {

    private final TalonSRX clawMotor;

    public Claw() {
        this.clawMotor = new TalonSRX(ClawConstants.motorDeviceId);
    }

    /**
     * Sets the speed of this claw, as a percentage between -1 and 1, with 1 being fully forwards and -1 being fully
     * backwards. Because the claw shouldn't require extremely precise movements and detection, complicated math
     * shouldn't really be required for this.
     * @param speed the new speed of the motor
     */
    public void setSpeed(double speed) {
        this.clawMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    /**
     * Opens the claw at the speed specified in {@link ClawConstants#openSpeed}.
     */
    public void open() {
        setSpeed(ClawConstants.openSpeed);
    }

    /**
     * Closes the claw at the speed specified in {@link ClawConstants#closeSpeed}.
     */
    public void close() {
        setSpeed(ClawConstants.closeSpeed);
    }

    /**
     * Stops the claw by setting its speed to zero.
     */
    public void stop() {
        setSpeed(0);
    }

}


