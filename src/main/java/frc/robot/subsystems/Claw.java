package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

/**
 * Subsystem for the claw that is positioned on the end of the pendulum.
 * Manages a motor and a simple limit switch (for knowing whether or not the
 * robot is holding anything).
 */
public class Claw extends SubsystemBase {

    private final ShuffleboardTab tab;

    private final TalonSRX clawMotor;
    private final DigitalInput limitSwitch;

    private boolean limitSwitchValue;

    public Claw() {
        this.clawMotor = new TalonSRX(ClawConstants.motorDeviceId);
        this.limitSwitch = new DigitalInput(ClawConstants.limitSwitchChannel);

        limitSwitchValue = limitSwitch.get();

        clawMotor.setNeutralMode(NeutralMode.Brake);
        clawMotor.configContinuousCurrentLimit(5);
        clawMotor.enableCurrentLimit(true);

        this.tab = Shuffleboard.getTab("Gripper");
        tab.addNumber("amps drawn", clawMotor::getStatorCurrent);
        tab.addBoolean("limit switch", limitSwitch::get);
        tab.add(this);
    }

    /**
     * Sets the amperage that the motor should draw.
     * @param speed the new speed of the motor
     */
    public void setAmperage(double speed) {
        this.clawMotor.set(TalonSRXControlMode.Current, speed);
    }

    /**
     * Stops the claw by setting its amperage to zero.
     */
    public void stop() {
        setAmperage(0);
    }

    /**
     * Updates the currently stored value of the limit switch.
     */
    public void updateLimitSwitch() {
        this.limitSwitchValue = switchPressed();
    }

    public void changeLimitSwitch(boolean value) {
        this.limitSwitchValue = value;
    }

    /**
     * Returns the most recently updated value of the limit switch. This should be used for when the switch is intended
     * to be updated at specific times instead of constantly.
     * @return the current stored value
     */
    public boolean guardedSwitchValue() {
        return limitSwitchValue;
    }

    /**
     * Retrieves information on whether or not the limit switch is currently being
     * pressed.
     * 
     * @return true if the switch is pressed
     */
    public boolean switchPressed() {
        return limitSwitch.get();
    }

}
