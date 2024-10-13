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
    private Limelight3A limelight;
    public List<List<Double>> corners;

    public int[] cornerIndices;
    double contourHeight;
    double contourWidth;
    public double[][] cornerPoints;
    public double deltaY;
    public double deltaX;
    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
//        telemetry.setMsTransmissionInterval(11); // Idk if we needs this
        limelight.pipelineSwitch(0); // or this3
        limelight.start();
        cornerPoints = new double[4][2];
    }

    public double getSampleOrientation() {

        LLResult result = limelight.getLatestResult();

        if (result!=null && result.isValid()) {
            List<LLResultTypes.ColorResult> colorRes = result.getColorResults();
            corners = colorRes.get(0).getTargetCorners();
            if (corners.size()>=4) {
                List<List<Double>> newCorners = colorRes.get(0).getTargetCorners().subList(0, 4);
                cornerIndices = sortCorners(newCorners);
                deltaY = Math.abs(newCorners.get(cornerIndices[0]).get(1) - newCorners.get(cornerIndices[1]).get(1));
                deltaX = Math.abs(newCorners.get(cornerIndices[0]).get(0) - newCorners.get(cornerIndices[1]).get(0));
//                for (int index = 0; index < 4; index++) {
//                    cornerPoints[index][0] = corners.get(cornerIndices[index]).get(0);
//                    cornerPoints[index][1] = corners.get(cornerIndices[index]).get(1);
//                }
                double angle = Math.toDegrees(Math.atan2(deltaY, deltaX));
                if (contourWidth > contourHeight) {
                    return 90 - angle;
                }
                return angle;
            }
        }
        return 0;
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
    public Pose3D getRobotPose() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            return result.getBotpose();
        }
        return null;
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

}
