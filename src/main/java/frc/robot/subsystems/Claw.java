package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

/**
 * Subsystem for the claw that is positioned on the end of the pendulum.
 * Manages a motor and a simple limit switch (for knowing whether or not the
 * robot is holding anything).
 */
public class Claw extends SubsystemBase {

    private final TalonSRX clawMotor;
    private final DigitalInput limitSwitch;

    public Claw() {
        this.clawMotor = new TalonSRX(ClawConstants.motorDeviceId);
        this.limitSwitch = new DigitalInput(ClawConstants.limitSwitchChannel);

        clawMotor.setNeutralMode(NeutralMode.Brake);
        clawMotor.configContinuousCurrentLimit(20);
        Shuffleboard.getTab("Arm").addNumber("amp", () -> clawMotor.getStatorCurrent());
    }

    /**
     * Sets the speed of this claw, as a percentage between -1 and 1, with 1 being
     * fully forwards and -1 being fully
     * backwards. Because the claw shouldn't require extremely precise movements and
     * detection, complicated math
     * shouldn't really be required for this.
     * 
     * @param speed the new speed of the motor
     */
    public void setSpeed(double speed) {
        this.clawMotor.set(TalonSRXControlMode.Current, speed);
    }

    /**
     * Stops the claw by setting its speed to zero.
     */
    public void stop() {
        setSpeed(0);
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
