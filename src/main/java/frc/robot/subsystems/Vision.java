package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoPositionConstants.AprilTagTransformDirection;
import frc.robot.Constants.AutoPositionConstants.GamePiecePlacementLevel;
import frc.robot.Constants.CameraConstants.CameraDefaults;

public class Vision extends SubsystemBase {
    public enum CameraType {
        None,
        Robot,
        Arm
    }

    private VideoSink cameraServer;
    private UsbCamera driverCam;
    private UsbCamera armCamera;

    private PhotonCamera apriltagCamLeft;
    private PhotonPoseEstimator estimatorLeft;

    private PhotonCamera apriltagCamRight;
    private PhotonPoseEstimator estimatorRight;

    private Transform3d centerToAprilTagCamera;

    private AprilTagFieldLayout tag_locations;
    private AprilTagTransformDirection selectedRobotTransform;
    private GamePiecePlacementLevel levelToPlace;

    private CameraType cameraWantedToDisplay;
    private CameraType cameraCurrentlyDisplayed;

    public Vision() {
        PhotonCamera.setVersionCheckEnabled(false);
        // this.cameraServer = CameraServer.getServer();

        // this.driverCam = CameraServer.startAutomaticCapture(0);
        // driverCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

        // this.armCamera = CameraServer.startAutomaticCapture(1);
        // driverCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

        this.apriltagCamLeft = new PhotonCamera("apriltagvisionleft");
        apriltagCamLeft.setDriverMode(false);

        this.apriltagCamRight = new PhotonCamera("apriltagvisionright");
        apriltagCamRight.setDriverMode(false);

        this.centerToAprilTagCamera = CameraDefaults.MountOne.getTransformation();

        this.selectedRobotTransform = AprilTagTransformDirection.CENTER;
        this.levelToPlace = GamePiecePlacementLevel.MIDDLE;

        this.cameraCurrentlyDisplayed = CameraType.None;
        this.cameraWantedToDisplay = CameraType.Robot;

        // assume that we are testing within our own facilities while testing, else use the current field resource file.
        if (Constants.testing) {
            tag_locations = new AprilTagFieldLayout(new ArrayList<AprilTag>(), 0, 0);
        } else {
            try {
                tag_locations = new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        if (DriverStation.getAlliance() == Alliance.Blue) {
            tag_locations.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } else {
            tag_locations.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        }

        this.estimatorLeft = new PhotonPoseEstimator(tag_locations, PoseStrategy.MULTI_TAG_PNP, apriltagCamRight, centerToAprilTagCamera.inverse());
        this.estimatorRight = new PhotonPoseEstimator(tag_locations, PoseStrategy.MULTI_TAG_PNP, apriltagCamLeft, centerToAprilTagCamera.inverse());

        // this.subsystemTab = Shuffleboard.getTab("Vision");
        // subsystemTab.add(CameraServer.startAutomaticCapture())
        // .withSize(6, 6)
        // .withPosition(0, 0);
    }

    @Override
    public void periodic() {
        // if (cameraWantedToDisplay != cameraCurrentlyDisplayed) {
        //     cameraCurrentlyDisplayed = cameraWantedToDisplay;
        //     if (cameraCurrentlyDisplayed == CameraType.Arm) {
        //         cameraServer.setSource(armCamera);
        //     }
        //     if (cameraCurrentlyDisplayed == CameraType.Robot) {
        //         cameraServer.setSource(driverCam);
        //     }
        // }
    }
    
    // uses optionals for optimal handling outside of the method.
    public List<Optional<EstimatedRobotPose>> getRobotPoseContributor() {
        return List.of(estimatorLeft.update(), estimatorRight.update());
    }

    public PhotonPipelineResult getCurrentCaptures() {
        return apriltagCamLeft.getLatestResult();
    }

    public boolean hasTargets() {
        return getCurrentCaptures().hasTargets();
    }

    public Pose3d getAprilTagPose(int id) {
        return tag_locations.getTagPose(id).get();
    }

    public void setPlacementSettings(GamePiecePlacementLevel level, AprilTagTransformDirection transform) {
        levelToPlace = level;
        selectedRobotTransform = transform;
    }

    public GamePiecePlacementLevel getLocationToPlace() {
        return levelToPlace;
    }

    public AprilTagTransformDirection getSelectedAprilTagTransform() {
        return selectedRobotTransform;
    }

    public void setDriverCamera(CameraType camera) {
        cameraWantedToDisplay = camera;
    }
}