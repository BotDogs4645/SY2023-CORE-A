package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class AttemptClawGrab extends CommandBase {

    private final Claw claw;
    private final long closeTimeMs, openTimeMs;
    private final double closeSpeed, openSpeed;

    private long endTime;
    private boolean switchStopped;

    public AttemptClawGrab(Claw claw, long closeTimeMs, double closeSpeed, long openTimeMs, double openSpeed) {
        this.claw = claw;

        this.closeTimeMs = closeTimeMs;
        this.closeSpeed = closeSpeed;
        this.openTimeMs = openTimeMs;
        this.openSpeed = openSpeed;

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        this.endTime = System.currentTimeMillis() + closeTimeMs;
        claw.setSpeed(closeSpeed);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        boolean limit = claw.switchPressed();

        // TODO: Wait time after the switch was pressed to work???
        if (limit) {
            switchStopped = true;
        }

        return limit || System.currentTimeMillis() >= endTime;
    }

    @Override
    public void end(boolean interrupted) {
        if (switchStopped) {
            // Stop claw. Nothing else should be necessary
            claw.setSpeed(0);
        } else {
            // Reopen claw
            new BasicClawControl(claw, openTimeMs, openSpeed).schedule();
        }
    }
}
