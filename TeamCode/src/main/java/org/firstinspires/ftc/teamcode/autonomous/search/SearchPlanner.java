package org.firstinspires.ftc.teamcode.autonomous.search;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Probability-driven planner. No hard-coded sequence — picks next cell by
 * utility = prior * decay / distanceCost. Keeps robot exploring where pollen
 * is statistically likely, not where code says to go.
 *
 * Tune priors in Dashboard after practice logs. Replace guess cells with
 * real field spawn coordinates (AndyMark CAD) on 2026-09-12.
 */
@Config
public class SearchPlanner {

    // ---- Tunables ----
    public static double DISTANCE_COST_PER_IN = 0.008; // penalize far cells slightly
    public static double NEIGHBOR_BOOST_RADIUS_IN = 18;
    public static double NEIGHBOR_BOOST = 1.4;

    private final List<SearchCell> cells = new ArrayList<>();

    public SearchPlanner() {
        // Preseason guess — replace with real. Coordinates assume ~70x70 field, origin at center or start.
        // We use start-relative poses; remap to field frame once localization is field-centric.
        // High priors near walls/corners as per BIOBUZZ preview: "Pollen will naturally roll against border, into corners"
        add("wall-n",  new Pose2d(24,  48, Math.toRadians(90)), 0.40);
        add("wall-s",  new Pose2d(24, -48, Math.toRadians(-90)),0.40);
        add("corner-ne", new Pose2d(48, 48, Math.toRadians(45)), 0.35);
        add("corner-nw", new Pose2d(-48,48, Math.toRadians(135)),0.35);
        add("corner-se", new Pose2d(48,-48, Math.toRadians(-45)),0.35);
        add("corner-sw", new Pose2d(-48,-48,Math.toRadians(-135)),0.35);
        add("line-mid", new Pose2d(0, 0, 0), 0.30); // piles/lines across field
        add("line-e",  new Pose2d(36, 0, 0), 0.28);
        add("line-w",  new Pose2d(-36,0, 0), 0.28);
        add("center", new Pose2d(0, 24, 0), 0.20);
    }

    private void add(String name, Pose2d pose, double prior) {
        cells.add(new SearchCell(name, pose, prior));
    }

    /** Full list for telemetry / custom. */
    public List<SearchCell> getCells() { return Collections.unmodifiableList(cells); }

    /** Replace guess cells with real field spawns — call once after kickoff. */
    public void setCells(List<SearchCell> real) {
        cells.clear(); cells.addAll(real);
    }

    public void addCell(SearchCell c) { cells.add(c); }

    /** Next best cell from current pose. Null if none. */
    public SearchCell next(Pose2d cur) {
        if (cells.isEmpty()) return null;
        SearchCell best = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        for (SearchCell c : cells) {
            double dx = c.pose.position.x - cur.position.x;
            double dy = c.pose.position.y - cur.position.y;
            double dist = Math.hypot(dx, dy);
            double score = c.utility() - dist * DISTANCE_COST_PER_IN;
            if (score > bestScore) { bestScore = score; best = c; }
        }
        return best;
    }

    /** Ranked list for debugging. */
    public List<SearchCell> ranked(Pose2d cur) {
        List<SearchCell> copy = new ArrayList<>(cells);
        copy.sort(Comparator.comparingDouble((SearchCell c) -> {
            double dist = Math.hypot(c.pose.position.x - cur.position.x, c.pose.position.y - cur.position.y);
            return -(c.utility() - dist * DISTANCE_COST_PER_IN);
        }));
        return copy;
    }

    public void markResult(SearchCell cell, boolean found) {
        if (cell == null) return;
        cell.markVisit(found);
        if (found) {
            // Boost neighbors — pollen often clusters (piles/lines)
            for (SearchCell other : cells) {
                if (other == cell) continue;
                double d = Math.hypot(other.pose.position.x - cell.pose.position.x,
                                      other.pose.position.y - cell.pose.position.y);
                if (d < NEIGHBOR_BOOST_RADIUS_IN) other.successBoost = NEIGHBOR_BOOST;
            }
        }
    }

    public void decayBoosts() {
        for (SearchCell c : cells) c.successBoost = Math.max(1.0, c.successBoost * 0.92);
    }

    // ---- Helper to build from field coords quickly ----
    public static SearchCell at(String name, double x, double y, double headingDeg, double prior) {
        return new SearchCell(name, new Pose2d(x, y, Math.toRadians(headingDeg)), prior);
    }
}
