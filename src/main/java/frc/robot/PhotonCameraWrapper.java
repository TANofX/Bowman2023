/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

import org.apache.commons.cli.Option;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import java.util.ArrayList;
import java.util.Optional;

public class PhotonCameraWrapper {
        public PhotonCamera photonCamera;
        public PhotonPoseEstimator photonPoseEstimator;
        public AprilTagFieldLayout atfl;
        public Translation2d center = new Translation2d(18.28, 0);
        public Translation2d rightpos = new Translation2d(18.28, -22);
        public Translation2d leftpos = new Translation2d(18.28, 22);

        public PhotonCameraWrapper() {
                // Set up a test arena of two apriltags at the center of each driver station set
                final AprilTag tag1 = new AprilTag(
                                1,
                                new Pose3d(new Pose2d(
                                                FieldConstants.length - FieldConstants.gridDepth,
                                                FieldConstants.closeWidth,
                                                Rotation2d.fromDegrees(180))));
                final AprilTag tag2 = new AprilTag(
                                2,
                                new Pose3d(new Pose2d(
                                                FieldConstants.length - FieldConstants.gridDepth,
                                                FieldConstants.closeMidWidth,
                                                Rotation2d.fromDegrees(180))));
                final AprilTag tag3 = new AprilTag(
                                3,
                                new Pose3d(new Pose2d(
                                                FieldConstants.length - FieldConstants.gridDepth,
                                                FieldConstants.midWidth,
                                                Rotation2d.fromDegrees(180))));
                final AprilTag tag4 = new AprilTag(
                                4,
                                new Pose3d(new Pose2d(
                                                FieldConstants.length - FieldConstants.humanStationDepth,
                                                FieldConstants.farWidth,
                                                Rotation2d.fromDegrees(180))));
                final AprilTag tag5 = new AprilTag(
                                5,
                                new Pose3d(new Pose2d(
                                                FieldConstants.humanStationDepth,
                                                FieldConstants.farWidth,
                                                Rotation2d.fromDegrees(0.0))));
                final AprilTag tag6 = new AprilTag(
                                6,
                                new Pose3d(new Pose2d(
                                                FieldConstants.gridDepth,
                                                FieldConstants.midWidth,
                                                Rotation2d.fromDegrees(0.0))));
                final AprilTag tag7 = new AprilTag(
                                7,
                                new Pose3d(new Pose2d(
                                                FieldConstants.gridDepth,
                                                FieldConstants.closeMidWidth,
                                                Rotation2d.fromDegrees(0.0))));
                final AprilTag tag8 = new AprilTag(
                                8,
                                new Pose3d(new Pose2d(
                                                FieldConstants.gridDepth,
                                                FieldConstants.closeWidth,
                                                Rotation2d.fromDegrees(0.0))));
                ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
                atList.add(tag1);
                atList.add(tag2);
                atList.add(tag3);
                atList.add(tag4);
                atList.add(tag5);
                atList.add(tag6);
                atList.add(tag7);
                atList.add(tag8);

                // TODO - once 2023 happens, replace this with just loading the 2023 field
                // arrangement
                atfl = new AprilTagFieldLayout(atList, FieldConstants.length, FieldConstants.width);
                
                // Forward Camera
                photonCamera = new PhotonCamera(
                                VisionConstants.cameraName); // Change the name of your camera here to whatever it is in
                                                             // the
                // PhotonVision UI.

                // Create pose estimator
                photonPoseEstimator = new PhotonPoseEstimator(
                                atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, VisionConstants.robotToCam);

                        }
        

        /**
         * @param estimatedRobotPose The current best guess at robot pose
         * @return
         * @return A pair of the fused camera observations to a single Pose2d on the
         *         field, and the time
         *         of the observation. Assumes a planar field and the robot is always
         *         firmly on the ground
         */
        public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
                photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
                return photonPoseEstimator.update();
        }

        public PhotonPipelineResult getTargets() {
                return photonCamera.getLatestResult();
        }
        public Optional <Pose2d> findTargetScoreLocation() {
                Pose2d targetPose = null;
                int tagid = photonCamera.getLatestResult().getBestTarget().getFiducialId();
                Optional <Pose3d> aprilTagCoordinate = atfl.getTagPose(tagid);
               
                if (aprilTagCoordinate.isPresent()) {
                        Translation2d tagPosition = new Translation2d(
                        aprilTagCoordinate.get().getX(),
                        aprilTagCoordinate.get().getY());
                        Translation2d targetPosition = center.plus(tagPosition);
                        Rotation2d rotation = new Rotation2d(aprilTagCoordinate.get().getRotation().getZ());
                        targetPose = new Pose2d(targetPosition, rotation);  


                }
                
             return Optional.ofNullable(targetPose);
        }
        
}
