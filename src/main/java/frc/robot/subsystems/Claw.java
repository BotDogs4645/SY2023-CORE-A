package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

/**
 * Subsystem for the claw that is positioned on the end of the pendulum.
 * Manages a motor and a simple limit switch (for knowing whether or not the robot is holding anything).
 */
public class Claw extends SubsystemBase {

    private final TalonSRX clawMotor;
    private final DigitalInput limitSwitch;

    private boolean lastSwitchPosition;

    public Claw() {
        this.clawMotor = new TalonSRX(ClawConstants.motorDeviceId);
        this.limitSwitch = new DigitalInput(ClawConstants.limitSwitchChannel);
    }

    /**
     * @return the motor controller instance itself
     */
    public TalonSRX getClawMotor() {
        return clawMotor;
    }

    /**
     * @return the limit switch input instance itself
     */
    public DigitalInput getLimitSwitch() {
        return limitSwitch;
    }

    /**
     * Updates the claw motor neutral mode, using the limit switch as a guide.
     */
    public void updateNeutral() {
        var newPosition = limitSwitch.get();

        if (newPosition != lastSwitchPosition) {
            var mode = newPosition ? NeutralMode.Brake : NeutralMode.Coast;
            this.clawMotor.setNeutralMode(mode);

            lastSwitchPosition = newPosition;
        }
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

    /**
     * Retrieves information on whether or not the limit switch is currently being pressed.
     * @return true if the switch is pressed
     */
    public boolean switchPressed() {
        return limitSwitch.get();
    }

}


