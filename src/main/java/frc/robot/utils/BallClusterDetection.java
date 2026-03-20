package frc.robot.utils;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class BallClusterDetection {

    static final double IOU_THRESHOLD = 0.10;
    static final double DIST_THRESHOLD = 45.0;

    // -----------------------------------------------------------------------
    // Box represented as int[4]: { xmin, ymin, xmax, ymax }
    // -----------------------------------------------------------------------

    /** Step 1: Convert 4 corner points to an axis-aligned box. */
    static int[] normalizeBox(int[][] corners) {
        int xmin = corners[0][0], xmax = corners[0][0];
        int ymin = corners[0][1], ymax = corners[0][1];
        for (int[] p : corners) {
            if (p[0] < xmin)
                xmin = p[0];
            if (p[0] > xmax)
                xmax = p[0];
            if (p[1] < ymin)
                ymin = p[1];
            if (p[1] > ymax)
                ymax = p[1];
        }
        return new int[] { xmin, ymin, xmax, ymax };
    }

    /** Step 2: Center of an axis-aligned box. Returns double[2]: { cx, cy }. */
    static double[] boxCenter(int[] box) {
        return new double[] {
                (box[0] + box[2]) / 2.0,
                (box[1] + box[3]) / 2.0
        };
    }

    /** Step 2: Area of an axis-aligned box. */
    static int boxArea(int[] box) {
        return (box[2] - box[0]) * (box[3] - box[1]);
    }

    /** Step 3A: Intersection over Union. */
    static double iou(int[] a, int[] b) {
        int interW = Math.max(0, Math.min(a[2], b[2]) - Math.max(a[0], b[0]));
        int interH = Math.max(0, Math.min(a[3], b[3]) - Math.max(a[1], b[1]));
        int interArea = interW * interH;
        int unionArea = boxArea(a) + boxArea(b) - interArea;
        if (unionArea == 0)
            return 0.0;
        return (double) interArea / unionArea;
    }

    /** Step 3B: Euclidean distance between box centers. */
    static double centerDistance(int[] a, int[] b) {
        double[] ca = boxCenter(a);
        double[] cb = boxCenter(b);
        double dx = ca[0] - cb[0];
        double dy = ca[1] - cb[1];
        return Math.sqrt(dx * dx + dy * dy);
    }

    /** Step 3: True if the two boxes satisfy either merge condition. */
    static boolean shouldMerge(int[] a, int[] b, double iouThreshold, double distThreshold) {
        return iou(a, b) > iouThreshold || centerDistance(a, b) < distThreshold;
    }

    /** Step 4: Enclosing bounding box of two boxes. */
    static int[] mergeBoxes(int[] a, int[] b) {
        return new int[] {
                Math.min(a[0], b[0]),
                Math.min(a[1], b[1]),
                Math.max(a[2], b[2]),
                Math.max(a[3], b[3])
        };
    }

    // -----------------------------------------------------------------------
    // Cluster: pairs a merged box with the set of original indices it contains.
    // -----------------------------------------------------------------------
    static class Cluster {
        int[] box;
        Set<Integer> indices;

        Cluster(int[] box, Set<Integer> indices) {
            this.box = box;
            this.indices = indices;
        }
    }

    /**
     * Step 5: Iteratively merge boxes, tracking original indices throughout.
     * Restarts from the beginning after every merge, stopping when a full
     * pass produces no merges.
     */
    static List<Cluster> iterativeMergeTracked(
            List<int[]> boxes, double iouThreshold, double distThreshold) {

        List<Cluster> clusters = new ArrayList<>();
        for (int i = 0; i < boxes.size(); i++) {
            Set<Integer> ids = new HashSet<>();
            ids.add(i);
            clusters.add(new Cluster(boxes.get(i), ids));
        }

        boolean merged = true;
        while (merged) {
            merged = false;
            outer: for (int i = 0; i < clusters.size(); i++) {
                for (int j = i + 1; j < clusters.size(); j++) {
                    Cluster ci = clusters.get(i);
                    Cluster cj = clusters.get(j);
                    if (shouldMerge(ci.box, cj.box, iouThreshold, distThreshold)) {
                        int[] combinedBox = mergeBoxes(ci.box, cj.box);
                        Set<Integer> combinedIds = new HashSet<>(ci.indices);
                        combinedIds.addAll(cj.indices);
                        clusters.remove(j);
                        clusters.remove(i);
                        clusters.add(i, new Cluster(combinedBox, combinedIds));
                        merged = true;
                        break outer;
                    }
                }
            }
        }
        return clusters;
    }

    /** Step 6: Cluster with the largest bounding-box area. */
    static Cluster selectTarget(List<Cluster> clusters) {
        Cluster best = null;
        int bestArea = -1;
        for (Cluster c : clusters) {
            int area = boxArea(c.box);
            if (area > bestArea) {
                bestArea = area;
                best = c;
            }
        }
        return best;
    }

    /**
     * From the members of the selected cluster, return the original index
     * whose center lies closest to the cluster's center.
     */
    static int mostCentralIndex(Cluster target, List<int[]> allBoxes) {
        double[] cc = boxCenter(target.box);
        int bestIdx = -1;
        double bestDist = Double.MAX_VALUE;
        for (int idx : target.indices) {
            double[] bc = boxCenter(allBoxes.get(idx));
            double dx = bc[0] - cc[0];
            double dy = bc[1] - cc[1];
            double d = Math.sqrt(dx * dx + dy * dy);
            if (d < bestDist) {
                bestDist = d;
                bestIdx = idx;
            }
        }
        return bestIdx;
    }

    /**
     * Full pipeline: corner-format detections → index of most central
     * detection in the largest cluster.
     *
     * @param rawDetections Each element is a 4×2 array of (x,y) corner points.
     * @param iouThreshold  Merge if IoU exceeds this (recommended 0.08–0.15).
     * @param distThreshold Merge if center distance is below this (recommended
     *                      40–50 px).
     * @return Index into rawDetections of the most central detection, or -1 if
     *         empty.
     */
    static int detectBallCluster(
            int[][][] rawDetections, double iouThreshold, double distThreshold) {

        if (rawDetections == null || rawDetections.length == 0)
            return -1;

        // Step 1: normalize corners to axis-aligned boxes
        List<int[]> boxes = new ArrayList<>();
        for (int[][] corners : rawDetections) {
            boxes.add(normalizeBox(corners));
        }

        // Step 5: merge iteratively
        List<Cluster> clusters = iterativeMergeTracked(boxes, iouThreshold, distThreshold);

        // Step 6: pick the largest cluster
        Cluster target = selectTarget(clusters);
        if (target == null)
            return -1;

        // Return the index of the most central original detection
        return mostCentralIndex(target, boxes);
    }

    /** Convenience overload using default thresholds. */
    public static int detectBallCluster(int[][][] rawDetections) {
        return detectBallCluster(rawDetections, IOU_THRESHOLD, DIST_THRESHOLD);
    }

    // -----------------------------------------------------------------------
    // Example usage
    // -----------------------------------------------------------------------
    public static void main(String[] args) {
        int[][][] sampleDetections = {
                { { 10, 20 }, { 50, 20 }, { 50, 60 }, { 10, 60 } }, // 0
                { { 45, 25 }, { 80, 25 }, { 80, 65 }, { 45, 65 } }, // 1
                { { 200, 150 }, { 230, 150 }, { 230, 180 }, { 200, 180 } }, // 2
                { { 215, 160 }, { 245, 160 }, { 245, 190 }, { 215, 190 } }, // 3
                { { 218, 155 }, { 248, 155 }, { 248, 185 }, { 218, 185 } }, // 4
        };

        int result = detectBallCluster(sampleDetections);
        System.out.println("Most central detection index: " + result);
    }
}