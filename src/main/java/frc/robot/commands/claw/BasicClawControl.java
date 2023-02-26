package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

/**
 * Simply runs the claw motor at a specified speed for a specified time. Due to there not being an encoder on our model,
 * the only metric for how long the motor should be run is time.
 */
public class BasicClawControl extends CommandBase {

    private final Claw claw;
    private final long timeMs;
    private final double speed;

    private long endTime;

    public BasicClawControl(Claw claw, long timeMs, double speed) {
        this.claw = claw;
        this.timeMs = timeMs;
        this.speed = speed;

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        this.endTime = System.currentTimeMillis() + timeMs;
        claw.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() >= endTime;
    }

    @Override
    public void end(boolean interrupted) {
        claw.setSpeed(0);
    }
}
