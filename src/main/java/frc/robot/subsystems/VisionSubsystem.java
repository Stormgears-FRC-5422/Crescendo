package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.vision.LimelightExtra;
import frc.utils.vision.LimelightHelpers;

import java.util.Arrays;
import java.util.Map;
import java.util.Optional;

import frc.robot.RobotState;

import static edu.wpi.first.math.util.Units.degreesToRadians;

public class VisionSubsystem extends SubsystemBase {
    private RobotState robotState;
    private String limelightId;
    private LimelightHelpers.LimelightResults latestLimelightResults = null;
    private int count;

    Optional<LimelightHelpers.LimelightTarget_Detector> noteDetector = Optional.empty();

    public VisionSubsystem(String limelightId) {
        this.limelightId = limelightId;
        LimelightHelpers.setLEDMode_PipelineControl("");
        LimelightHelpers.setLEDMode_ForceBlink("");
        robotState = RobotState.getInstance();
        count = 0;
    }

    public LimelightHelpers.LimelightResults getLatestResults() {
        if (latestLimelightResults == null) {
            latestLimelightResults = LimelightHelpers.getLatestResults(limelightId);
            count += 1;
            if (count % 50 == 0) {
                //    System.out.println("info : " + latestLimelightResults.);
            }

        }
        return latestLimelightResults;
    }

    public void addDetectorDashboardWidgets(ShuffleboardTab layout) {
        layout.addBoolean("Target", () -> getLatestDetectorTarget().isPresent()).withPosition(0, 0);
        layout.addDouble("Tx", () -> {
            var optResults = getLatestDetectorTarget();
            if (optResults.isPresent()) {
                return optResults.get().tx;
            }
            return 0;
        }).withPosition(0, 1);
        layout.addDouble("Ty", () -> {
            var optResults = getLatestDetectorTarget();
            if (optResults.isPresent()) {
                return optResults.get().ty;
            }
            return 0;
        }).withPosition(0, 2);
        layout.addString("Class", () -> {
            var optResults = getLatestDetectorTarget();
            if (optResults.isPresent()) {
                return optResults.get().className;
            }
            return "";
        }).withPosition(0, 3);
    }

    public Optional<LimelightHelpers.LimelightTarget_Detector> getLatestDetectorTarget() {
        var results = getLatestResults();
        var targetResult = results;
//        System.out.println(results.valid);
//        System.out.println(Arrays.toString(results.targets_Detector));
//        System.out.println(results.targets_Detector.length);

        if (targetResult != null && targetResult.valid && targetResult.targets_Detector.length > 0) {
            return Optional.of(targetResult.targets_Detector[0]);
        }
        return Optional.empty();
    }

    public boolean isNoteDetected() {
        var results = getLatestResults();


        if (results == null) {
            return false;
        }
        var targetResult = results;

        if (targetResult != null && targetResult.valid && targetResult.targets_Detector.length > 0) {
            boolean isPresent = Optional.of(targetResult.targets_Detector[0]).isPresent();
            robotState.setIsNoteDetected(isPresent);
            return isPresent;
        }
        robotState.setIsNoteDetected(false);
        return false;
    }

    public Optional<LimelightHelpers.LimelightTarget_Retro> getLatestRetroTarget() {
        var results = getLatestResults();
        if (results == null) {
            return Optional.empty();
        }
        var targetResult = results;
        if (targetResult != null && targetResult.valid && targetResult.targets_Retro.length > 0) {
            return Optional.of(targetResult.targets_Retro[0]);
        }
        return Optional.empty();
    }

    public Optional<LimelightHelpers.LimelightTarget_Fiducial> getLatestFiducialsTarget() {
        var results = getLatestResults();
        if (results == null) {
            return Optional.empty();
        }
        var targetResult = results;
        if (targetResult != null && targetResult.valid && targetResult.targets_Fiducials.length > 0) {
            return Optional.of(targetResult.targets_Fiducials[0]);
        }
        return Optional.empty();
    }

    private static Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) {
            //System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(degreesToRadians(inData[5]));
//        AprilTagPoseEstimator estimator = new AprilTagPoseEstimator();
        return new Pose2d(tran2d, r2d);
    }




    @Override
    public void periodic() {
        latestLimelightResults = null;
//        RobotState.getInstance().setVisionPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight"),
//            LimelightHelpers.getTV("limelight"));
        isNoteDetected();
    }

    public double getTX() {
        return LimelightExtra.getTX(limelightId);
    }

    public double getTY() {
        return LimelightExtra.getTY(limelightId);
    }

    public boolean getValid() {
        return LimelightExtra.hasValidTarget(limelightId);
    }

}
