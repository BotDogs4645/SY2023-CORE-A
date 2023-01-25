package frc.robot.commands.swervecommands;

import frc.bdlib.driver.JoystickAxisAIO;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.swervehelper.SwerveSettings;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerveAroundPoint extends CommandBase {
    
    private Swerve s_Swerve;

    private JoystickAxisAIO x;
    private JoystickAxisAIO y;
    private JoystickAxisAIO r;

    /**
     * Driver control
     */
    public TeleopSwerveAroundPoint(Swerve s_Swerve, JoystickAxisAIO x, JoystickAxisAIO y, JoystickAxisAIO r) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.x = x;
        this.y = y;
        this.r = r;
    }

    @Override
    public void execute() {
        s_Swerve.pointOrientedDrive(
            new Translation2d(-y.getValue(), -x.getValue()).times(SwerveSettings.driver.maxSpeed()),
            r.getValue() * SwerveSettings.driver.maxRotationSpeed(),
            new Translation2d(SwerveSettings.SwerveDriveTrain.wheelBase / 2.0, SwerveSettings.SwerveDriveTrain.trackWidth / 2.0)
          );
    }
}
