package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Camlight {
    private final double sampleHeight = 3.8;
    private final double sampleWidth = 8.9;

    private final double maxWHRatio = 2.275;
    private final double minWHRatio = 0.44;
    private Limelight3A limelight;
    public List<List<Double>> corners;
    public String cornerList;

    // Sample Detection and April Tag Detection
    public double tX, tY, targetArea;
    public boolean targetFound;
    public int[] cornerIndices;
    public double contourHeight;
    public double contourWidth;
    public double[][] cornerPoints;
    public double deltaY;
    public double deltaX;
    public boolean tiltedLeft;
    int pipelineIndex;

    private enum Pipeline {
        YellowSample(0), NeuralDetector(5), AprilTag(7);
        private final int index;
        Pipeline(int val) {
            this.index = val;
        }

        public int getPipelineIndex() {
            return index;
        }
    }

    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        pipelineIndex = Pipeline.YellowSample.index;
//        telemetry.setMsTransmissionInterval(11); // Idk if we needs this
        limelight.pipelineSwitch(Pipeline.YellowSample.index); // or this3
        limelight.start();
        cornerPoints = new double[4][2];
    }

    public double getSampleOrientation() {
        targetFound = false;
        if (pipelineIndex != Pipeline.YellowSample.index) {
            pipelineIndex = Pipeline.YellowSample.index;
            limelight.pipelineSwitch(Pipeline.YellowSample.index);
        }

        LLResult result = limelight.getLatestResult();
        tX = 0;
        tY = 0;
        targetArea = 0;
        if (result!=null && result.isValid()) {
            List<LLResultTypes.ColorResult> colorRes = result.getColorResults();
            if (colorRes.size()==0) {
                return -1;
            }
            corners = colorRes.get(0).getTargetCorners();

            if (corners.size()>=4) {
                targetFound = true;
                List<List<Double>> newCorners = colorRes.get(0).getTargetCorners().subList(0, 4);
                tX = colorRes.get(0).getTargetXDegrees();
                tY = colorRes.get(0).getTargetYDegrees();
                cornerIndices = sortCorners(newCorners);
                targetArea = colorRes.get(0).getTargetArea();
                if (newCorners.get(cornerIndices[0]).get(0)-newCorners.get(cornerIndices[1]).get(0)
                    > newCorners.get(cornerIndices[2]).get(1)-newCorners.get(cornerIndices[1]).get(1)) {
                    deltaY = newCorners.get(cornerIndices[0]).get(1) - newCorners.get(cornerIndices[1]).get(1);
                    deltaX = newCorners.get(cornerIndices[0]).get(0) - newCorners.get(cornerIndices[1]).get(0);
                }
                else {
                    deltaY = newCorners.get(cornerIndices[2]).get(1) - newCorners.get(cornerIndices[1]).get(1);
                    deltaX = newCorners.get(cornerIndices[2]).get(0) - newCorners.get(cornerIndices[1]).get(0);
                }

//                for (int index = 0; index < 4; index++) {
//                    cornerPoints[index][0] = corners.get(cornerIndices[index]).get(0);
//                    cornerPoints[index][1] = corners.get(cornerIndices[index]).get(1);
//                }

                double angle = Math.toDegrees(Math.atan2(deltaY, deltaX));
                if (angle < 0) {
                    return 180+angle;
                }
                return angle;
            }
        }
        return -1;
    }

    public double getSampleOrientationNN() {
        targetFound = false;
        getSampleOrientation();
        if (pipelineIndex != Pipeline.NeuralDetector.index) {
            pipelineIndex = Pipeline.NeuralDetector.index;
            limelight.pipelineSwitch(Pipeline.NeuralDetector.index);
        }

        LLResult result = limelight.getLatestResult();
        double sampleHeight = 8.9;
        double sampleWidth = 3.8;
        if (result!=null && result.isValid() && result.getDetectorResults().size()>0) {
            LLResultTypes.DetectorResult detectorResult = result.getDetectorResults().get(0);
            corners = detectorResult.getTargetCorners();
            if (corners.size() == 4) {
                targetFound = true;
                contourWidth = corners.get(0).get(0) - corners.get(1).get(0);
                contourHeight = corners.get(0).get(1) - corners.get(2).get(1);
                double widthHeightRatio = contourWidth/contourHeight;
                double angle = Math.signum(widthHeightRatio) *
                        Math.toDegrees(Math.atan((sampleHeight - (sampleWidth*widthHeightRatio))/
                        ((sampleHeight*widthHeightRatio)-sampleWidth)));

                if (tiltedLeft) {
                    return 180-angle;
                }
                else {
                    return angle;
                }
            }
        }
        return -1;
    }


    private int[] sortCorners(List<List<Double>> corners) {
        int leastYIndex = 0;
        int secondLeastYIndex = 1;

        for (int i = 0; i<corners.size(); i++) {
            double val = corners.get(i).get(1);
            if (val < corners.get(leastYIndex).get(1)) {
                secondLeastYIndex = leastYIndex;
                leastYIndex = i;
            }
            else if (val < corners.get(secondLeastYIndex).get(1) && i!=leastYIndex) {
                secondLeastYIndex = i;
            }
        }
        int bottomRight, bottomLeft;
        if (corners.get(leastYIndex).get(0) > corners.get(secondLeastYIndex).get(0)) {
            bottomRight = leastYIndex;
            bottomLeft = secondLeastYIndex;
        }
        else {
            bottomRight = secondLeastYIndex;
            bottomLeft = leastYIndex;
        }
        int topRight, topLeft;
        int [] remainingIndices = findRemainingIntegers(bottomLeft, bottomRight);
        if (corners.get(remainingIndices[0]).get(0) > corners.get(remainingIndices[1]).get(0)) {
            topRight = remainingIndices[0];
            topLeft = remainingIndices[1];
        }
        else {
            topRight = remainingIndices[1];
            topLeft = remainingIndices[0];
        }
        contourHeight = Math.abs(corners.get(topRight).get(1)-corners.get(bottomRight).get(1)); // Abs not really needed
        contourWidth = Math.abs(corners.get(bottomRight).get(0)-corners.get(bottomLeft).get(0)); // Abs not really needed


        return new int[]{bottomRight, bottomLeft, topLeft, topRight};
    }
    public int[] findRemainingIntegers(int given1, int given2) {
        // Set of all integers between 0 and 3
        Set<Integer> allIntegers = new HashSet<>(Arrays.asList(0, 1, 2, 3));

        // Remove the two given integers
        allIntegers.remove(given1);
        allIntegers.remove(given2);

        // Convert the remaining integers to an array
        int[] remaining = new int[2];
        int index = 0;
        for (int num : allIntegers) {
            remaining[index++] = num;
        }

        return remaining;
    }

    public Pose3D detectAprilTag() {
        targetFound = false;
        if (pipelineIndex != Pipeline.AprilTag.index) {
            pipelineIndex = Pipeline.AprilTag.index;
            limelight.pipelineSwitch(Pipeline.AprilTag.index);
        }
        tX = 0;
        tY = 0;
        targetArea = 0;

        LLResult result = limelight.getLatestResult();
        if (result!=null && result.isValid() && result.getFiducialResults().size()>0) {
            targetFound = true;
            LLResultTypes.FiducialResult tag = result.getFiducialResults().get(0);
            tX = tag.getTargetXDegrees();
            tY = tag.getTargetYDegrees();
            targetArea = tag.getTargetArea();
        }
        return null;
    }

}
