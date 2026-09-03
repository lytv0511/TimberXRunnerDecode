package org.firstinspires.ftc.teamcode.autonomous.search;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * Discrete cell for probability field.
 * BIOBUZZ pollen priors: walls/corners > field lines > piles > center.
 * Before kickoff we guess; after kickoff replace with real field spawn coords.
 */
public class SearchCell {
    public final String name;
    public final Pose2d pose; // field-centric pose to drive to for scanning
    public final double prior; // 0..1, designer belief pollen is here
    public double successBoost = 1.0; // >1 if neighbor just scored
    public int visits = 0;
    public int failures = 0;
    public long lastVisitMs = 0;

    public SearchCell(String name, Pose2d pose, double prior) {
        this.name = name;
        this.pose = pose;
        this.prior = prior;
    }

    /** Utility: prior decayed by visit history and recency. */
    public double utility() {
        double decay = Math.pow(0.35, failures); // each failure 65% drop
        // freshness: if visited <2s ago, suppress (avoid oscillation)
        double freshness = (System.currentTimeMillis() - lastVisitMs < 2000) ? 0.25 : 1.0;
        return prior * decay * successBoost * freshness;
    }

    public void markVisit(boolean found) {
        visits++;
        lastVisitMs = System.currentTimeMillis();
        if (!found) failures++;
        else {
            // found → nearby cells deserve boost (handled by planner)
            failures = Math.max(0, failures - 1);
        }
    }

    @Override public String toString() { return name + " pri=" + prior + " util=" + String.format("%.2f", utility()); }
}
