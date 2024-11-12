package org.firstinspires.ftc.teamcode.util;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Iterator;

public class RunningStats {
    private final int windowSize;
    private final double[] window;
    private double mean;
    private double zScoreThreshold;
    private double variance;
    private double stdDev;
    private int count;

    public RunningStats(int windowSize, double zScoreThreshold) {
        this.zScoreThreshold = zScoreThreshold;
        this.windowSize = windowSize;
        this.window = new double[windowSize];
        this.mean = 0.0;
        this.count = 0;
    }

    // Method to update the running mean and standard deviation with a new data point
    public void addDataPoint(double newDataPoint) {
        // If the window is full, remove the oldest data point
        int index = count % windowSize;

        if (count >= windowSize) {
            double oldMean = mean;
            double toDelete = window[0];
            double newMean = oldMean + (newDataPoint-toDelete)/windowSize;

        }

    }

    // Update mean and variance when adding a new data point
    private void updateStatisticsOnAdd(double newDataPoint) {
//        int n = window.size();

        // Calculate the new mean
        double oldMean = mean;
//        mean += (newDataPoint - oldMean) / n;

        // Update sum of squared differences for variance/std dev
//        sumSquaredDifferences += (newDataPoint - mean) * (newDataPoint - oldMean);
    }

    // Update mean and variance when removing the oldest data point
    private void updateStatisticsOnRemove(double oldDataPoint) {
//        int n = window.size();
//
//        if (n == 0) return;  // Avoid division by zero
//
//        // Calculate the new mean after removing the old data point
//        double oldMean = mean;
//        mean -= (oldDataPoint - oldMean) / n;
//
//        // Update sum of squared differences for variance/std dev
//        sumSquaredDifferences -= (oldDataPoint - mean) * (oldDataPoint - oldMean);
    }

    // Method to get the current mean
    public double getMean() {
        int currentSize = Math.min(count, windowSize);
//        return sum/currentSize;
        return 0;
    }

    public String printQ() {
        String s = "[";
        for (Double val : window) {
            s = s.concat(val + ", ");
        }
        return s.concat("]");
    }

    // Method to get the current standard deviation
    public double getStandardDeviation() {
//        if (window.size() < 2) {
//            return 0.0; // Not enough data points for std deviation
//        }
//        return Math.sqrt(sumSquaredDifferences / (window.size()));
        return 0;
    }

    public boolean isOutlier(double val) {
        double stdDev = getStandardDeviation();
        if (stdDev == 0)
            return false;

        double zScore = (val-mean) / stdDev;
        return Math.abs(zScore) > zScoreThreshold;
    }
}
