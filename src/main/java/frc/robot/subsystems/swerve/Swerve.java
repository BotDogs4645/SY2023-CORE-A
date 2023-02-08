package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.swervehelper.CTREModuleState;
import frc.robot.util.swervehelper.SwerveSettings;
import frc.robot.util.swervehelper.SwerveSettings.PathList;
import frc.robot.util.swervehelper.SwerveSettings.SwerveDriveTrain;
import frc.robot.util.swervehelper.SwerveSettings.ShuffleboardConstants.BoardPlacement;

public class Swerve extends SubsystemBase {
    public record ChassisControlRequest(Swerve swerve, Pose2d posReq, boolean openLoop, double power) {
        static SwerveModuleState[] states;

        public ChassisControlRequest {
            // Second order kinematics - kind of
            // Twist2d requestedRobotPose = posReq.log(new Pose2d(
            //     swerve.speedVector.vxMetersPerSecond * 0.020,
            //     swerve.speedVector.vyMetersPerSecond * 0.020,
            //     Rotation2d.fromRadians(swerve.speedVector.omegaRadiansPerSecond * 0.020)
            // ));

            states = SwerveDriveTrain.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    posReq.getX(),
                    posReq.getY(),
                    posReq.getRotation().getRadians(),
                    swerve.getYaw()
                )
            );
            SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveSettings.driver.maxSpeed());
        }

        public SwerveModuleState[] getRequestInStates() {
            return states;
        }
    };

    public NetworkTable nTable = Constants.LogTable.getSubTable("Swerve");

    private HashMap<String, Command> events = new HashMap<String, Command>();
    private HashMap<PathList, PathPlannerTrajectory> trajectories = new HashMap<PathList, PathPlannerTrajectory>();
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private WPI_Pigeon2 gyro;
    private ShuffleboardTab sub_tab;
    private SwerveAutoBuilder builder;
    private double chassis_speed; // meters / second
    private Field2d field;

    public ChassisSpeeds speedVector;

    /**
     * A swerve implementation using MK4 SDS modules, with full field oriented features.<p>
     * Original code from Team 364, heavily modified by Dave and Aidan
     * @return Swerve Drive subsystem
     */
    public Swerve() {
        // Declares and resets the Gyro to default. This wipes all settings about the gyro,
        // making it customizable in code only.
        this.gyro = new WPI_Pigeon2(SwerveDriveTrain.pigeonID, "canivore");
        gyro.configFactoryDefault();

        zeroGyro();

        // Used in the chassis speed calculation, check update() for more info
        this.chassis_speed = 0.0;

        // Gets us the swerve tab.
        this.sub_tab = Shuffleboard.getTab("swerve_tab");

        // These are our swerve modules. Each module has it's own constants
        // We also port the subsystem tab straight there so they can add their own information
        // We store them in an array so we can iterate through at any point.
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveDriveTrain.Mod0.constants, sub_tab),
            new SwerveModule(1, SwerveDriveTrain.Mod1.constants, sub_tab),
            new SwerveModule(2, SwerveDriveTrain.Mod2.constants, sub_tab),
            new SwerveModule(3, SwerveDriveTrain.Mod3.constants, sub_tab)
        };

        // SwerveDrivePoseEstimator instances are used to calculate and keep track of the Robot's
        // pose, which is essentially the coordinates and orientation of the robot.
        this.swerveOdometry = new SwerveDriveOdometry(SwerveDriveTrain.swerveKinematics, getYaw(), getModulePositions());

        // The SwerveAutoBuilder is used to create paths for this particular swerve drive.
        // All the PID is contained here, and no other commands relating to PathPlanner have to be created.
        // This builder also uses events, which is explained in the JavaDocs for the addEvents command.
        this.builder = new SwerveAutoBuilder(
            this::getPose, // Pose2d supplier
            this::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            SwerveDriveTrain.swerveKinematics, // SwerveDriveKinematics
            new PIDConstants(2.5, 0.0, 0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            this::setModuleStates, // Module states consumer used to output to the drive subsystem
            events,
            this // The drive subsystem. Used to properly set the requirements of path following commands
        );

        // Gyro initialization, we use a Pigeon and we want the Yaw in degrees so we can directly
        // show it on Shuffleboard.
        sub_tab.add("Pigeon IMU", gyro)
        .withSize(2, 3)
        .withPosition(4, 3)
        .withWidget(BuiltInWidgets.kGyro);

        // Our speedometer, uses the chassis_speed variable.
        sub_tab.addDouble("Chassis Speedometer: MPS", this::getChassisSpeed)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", 0.0, "Max", SwerveSettings.driver.maxSpeed(), "Show value", true))
        .withSize(4, 3)
        .withPosition(3, 0);

        this.field = new Field2d();

        ShuffleboardTab tab = Shuffleboard.getTab("Field");
        tab.add("Field", field)
        .withSize(7, 4)
        .withWidget(BuiltInWidgets.kField);

        // Add RPM and current calculations for each module and place them on the Shuffleboard
        for (int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule cur = mSwerveMods[i];
            BoardPlacement placement = BoardPlacement.valueOf("RPM" + i);
            ShuffleboardLayout layout = sub_tab.getLayout("mod " + cur.moduleNumber, BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 1, "Number of rows", 2, "Label Position", "TOP"))
            .withPosition(placement.getX(), placement.getY())
            .withSize(2, 3);

            layout.addDouble("RPM " + i, () -> Math.abs(cur.mDriveMotor.getObjectRotationsPerMinute()))
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min", 0, "Max", 800, "Show value", true));

            layout.addDouble("AMPS " + i, () -> cur.mDriveMotor.getSupplyCurrent())
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("Min", 0, "Max", SwerveDriveTrain.drivePeakCurrentLimit));
        }

        // iterates through the available path enums, and then puts them into the available path
        // hashmap.
        for (PathList enu: PathList.values()) {
            trajectories.put(enu, PathPlanner.loadPath(enu.toString(), enu.getConstraints()));
        }
    }

    /**
     * Used to port joystick input from the Drive command into the system.
     * @param translation The value of requested meters of translation from the current point. 
     * @param rotation The value of requested rotation in radians
     * @param fieldRelative Determines whether or not to establish field oriented control
     * @param isOpenLoop Determines whether or not to use PID for values; true = yes, false = no.
     * 
     */
    public void drive(ChassisControlRequest request) {
        // How module states work is this: we have a current position, a translation that we want to do, and a rotation vector
        // that we also want to do. From there, we take our current position add the translation and rotation and using
        // inverse kinematics, it returns each module's "state", or rather what direction to rotate to and what velocity to
        // spin at.
        setModuleStates(request.getRequestInStates(), request.openLoop(), request.power());
    }

    /**
     * Sets each module state, requires all states. This version always has openLoop at false,
     * because it is used for autonomous, which requires PID.
     * @param desiredStates Array of module states 
     * 
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveSettings.driver.maxSpeed());
        setModuleStates(desiredStates, false, 1.0);
    }

    /**
     * Sets each module state, requires all states. This version has openLoop as optional, which
     * means it can be used with a joystick.
     * @param desiredStates Array of module states
     * @param isOpenLoop Determines if it uses PID
     * 
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop, double powerPercentage) {
        nTable.putValue(
            "Swerve Desired States",
            NetworkTableValue.makeDoubleArray(CTREModuleState.getModuleStatesExpanded(desiredStates))
        );

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop, powerPercentage);
        }
    }

    /**
     * Gets the current {@link Pose2d}, with all translation values in meters.
     * @return the current pose in meters
     */
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    /**
     * Resets the odometry to a specified {@link Pose2d}.
     * @param pose Resets the robot's pose to this pose
     */
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /**
     * Gets the position of all the modules
     * @return Returns every swerve module's position in {@link SwerveModulePosition} form.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getSwervePosition();
        }
        return states;
    }

    /**
     * Resets the gyro, used for FOC.
     */
    public void zeroGyro(){
        gyro.setYaw(0);
    }

    /**
     * Gets the chassis's speed
     * @return chassis speed in meters / second
     */
    public double getChassisSpeed() {
        return chassis_speed;
    }

    /**
     * Gets the yaw with special math to make the gyro non-continous<p>
     * Reason why is because CTRE does not do continious, while WPILib does.
     * @return Rotation2d representing the yaw
     */
    public Rotation2d getYaw() {
        return gyro.getRotation2d();
    }

    public void resetDirection() {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), new Pose2d());
    }

    /**
     * Adds an event to the event list. The event list directly corresponds to the strings you give
     * events in the PathPlanner application.
     * @param event_name The same name as the one you assigned in PathPlanner
     * @param command_to_execute The {@link Command} you want to execute at this event fire
     */
    public void addEvent(String event_name, Command command_to_execute) {
        events.put(event_name, command_to_execute);
    }

    /**
     * Used to get a path command from only one PathList trajectory. This variant is to get
     * the Command for the first path, is_first has to be true.
     * @param traj_path The {@link PathList} trajectory from the established list of paths to use
     * @param is_first The variable that determines whether or not to reset the gyro at the start.
     * @return the usable {@link Command}
     * 
     */
    public Command getSoloPathCommand(PathList traj_path, boolean is_first) {
        PathPlannerTrajectory traj = trajectories.get(traj_path);
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
              // Reset odometry for the first path you run during auto
              if(is_first){
                  this.resetOdometry(traj.getInitialHolonomicPose());
              }
            }),
            builder.followPathWithEvents(traj)
        );
    }

    /**
     * Used to get a path command from only one PathList trajectory.
     * @param path The {@link PathList} trajectory from the established list of paths to use
     * @return the usable {@link Command} 
     */
    public Command getSoloPathCommand(PathList path) {
        return builder.followPathWithEvents(trajectories.get(path));
    }

    /**
     * Used to get a single command that runs all supplied trajectory {@link PathList} trajectories.<p>
     * DOES NOT reset odometry at the beginning.
     * @param traj A list of {@link PathList} trajectories to create a {@link Command} from
     * @return the usable {@link Command}
     */
    public Command getFullAutoPath(PathList... traj) {
        ArrayList<PathPlannerTrajectory> paths = new ArrayList<PathPlannerTrajectory>();
        for (int i = 0; i < traj.length; i++) {
            paths.add(trajectories.get(traj[i]));
        }

        return builder.fullAuto(paths);
    }

    public void provideVisionInformation(Pose2d suspected_pose) {
        if (suspected_pose.getTranslation().getDistance(getPose().getTranslation()) < 1) {
            // seems close enough
            //swerveOdometry.addVisionMeasurement(suspected_pose, Timer.getFPGATimestamp());
        }
    }

    public ChassisControlRequest generateRequest(Pose2d posReq, boolean openLoop, double power) {
        return new ChassisControlRequest(this, posReq, openLoop, power);
    }

    @Override
    public void periodic() {
        // updates our global swerve odometry, making them fully available for use throughout the sub.
        swerveOdometry.update(getYaw(), getModulePositions());

        field.setRobotPose(getPose());

        // state collection
        SwerveModuleState currentStates[] = new SwerveModuleState[mSwerveMods.length];
        for (int i = 0; i < mSwerveMods.length; i++) {
            currentStates[i] = mSwerveMods[i].getState();
        }

        nTable.putValue(
            "Swerve Current States",
            NetworkTableValue.makeDoubleArray(CTREModuleState.getModuleStatesExpanded(currentStates))
        );

        nTable.putValue("Yaw", NetworkTableValue.makeDouble(getYaw().getDegrees()));

        this.speedVector = 
            SwerveDriveTrain.swerveKinematics.toChassisSpeeds(
                currentStates
            );
        
        // considering x and y are orthogonal, we can just use the pythagorean theorem
        // to add the vectors together and get the chassis_speed in m/s.
        chassis_speed = Math.sqrt(Math.pow(speedVector.vxMetersPerSecond, 2) + Math.pow(speedVector.vyMetersPerSecond, 2));
    }
}