package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoPositionConstants.PlacementLocation;
import frc.robot.Constants.AutoPositionConstants.PoseAlignment;
import frc.robot.Constants.CameraConstants.CameraDefaults;

public class Vision extends SubsystemBase {

    private PhotonCamera driver_cam;
    private PhotonCamera apriltag_cam;
    private Transform3d centerToAprilTagCamera;

    private AprilTagFieldLayout tag_locations;
    private PoseAlignment selectedRobotPosition;
    private PlacementLocation selectedLocationToPlace;

    public Vision() {
        this.driver_cam = new PhotonCamera("drivervision");
        driver_cam.setDriverMode(true);

        this.apriltag_cam = new PhotonCamera("apriltagvision");
        apriltag_cam.setDriverMode(false);
        this.centerToAprilTagCamera = CameraDefaults.MountOne.getTransformation();

        this.selectedRobotPosition = PoseAlignment.CENTER;


        // assume that we are testing within our own facilities while testing, else use the current field resource file.
        if (Constants.testing) {
            tag_locations = new AprilTagFieldLayout(new ArrayList<AprilTag>(), 0, 0);
        } else {
            try {
                tag_locations = new AprilTagFieldLayout(AprilTagFields.k2022RapidReact.m_resourceFile);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        if (DriverStation.getAlliance() == Alliance.Blue) {
            tag_locations.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } else {
            tag_locations.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        }
    }

    @Override
    public void periodic() {}
    
    // uses optionals for optimal handling outside of the method.
    public Optional<Pose2d> getRobotPoseContributor() {
        PhotonPipelineResult results = getCurrentCaptures();
        if (results.hasTargets()) {
            PhotonTrackedTarget bestTarget = results.getBestTarget();
            if (bestTarget.getPoseAmbiguity() < .1) {
                // good capture
                Pose3d fieldRelativeAprilTagPose = tag_locations.getTagPose(bestTarget.getFiducialId()).get();
                Pose2d calculatedRobotPose = 
                    fieldRelativeAprilTagPose
                        .transformBy(bestTarget.getBestCameraToTarget().inverse())
                        .transformBy(centerToAprilTagCamera.inverse())
                        .toPose2d();
                return Optional.of(calculatedRobotPose);
            }
        }
        return Optional.empty();
    }

    public PhotonPipelineResult getCurrentCaptures() {
        return apriltag_cam.getLatestResult();
    }

    public boolean hasTargets() {
        return getCurrentCaptures().hasTargets();
    }

    public Pose3d getAprilTagPose(int id) {
        return tag_locations.getTagPose(id).get();
    }

    public void setPlacementSettings(PlacementLocation location, PoseAlignment alignment) {
        selectedLocationToPlace = location;
        selectedRobotPosition = alignment;
    }

    public PlacementLocation getLocationToPlace() {
        return selectedLocationToPlace;
    }

    public PoseAlignment getPoseAlignment() {
        return selectedRobotPosition;
    }
}