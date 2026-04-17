package aebs.simulator.scenario;

import aebs.simulator.model.Aabb;
import aebs.simulator.model.Vec2;
import aebs.simulator.world.CarBlock;
import aebs.simulator.world.PedestrianBlock;
import aebs.simulator.world.WorldState;

import java.util.Random;

/**
 * Simple scenario engine that spawns NPC "car blocks" ahead of the ego vehicle and moves them.
 * Coordinate system: origin at top-left; +x right, +y down.
 */
public final class ScenarioEngine {
    private final Random rng;
    private final double laneXCenter;
    private final double npcSpawnY;
    private double spawnCooldownS = 0.0;
    private int npcSeq = 0;

    private double pedSpawnCooldownS = 0.0;
    private int pedSeq = 0;

    public ScenarioEngine(long seed, double laneXCenter, double npcSpawnY) {
        this.rng = new Random(seed);
        this.laneXCenter = laneXCenter;
        this.npcSpawnY = npcSpawnY;
    }

    public void step(WorldState w, double dtSeconds) {
        spawnCooldownS = Math.max(0.0, spawnCooldownS - dtSeconds);
        pedSpawnCooldownS = Math.max(0.0, pedSpawnCooldownS - dtSeconds);
        if (spawnCooldownS <= 0.0) {
            maybeSpawnNpc(w);
        }
        if (pedSpawnCooldownS <= 0.0) {
            maybeSpawnPedestrian(w);
        }

        // Move all cars
        w.ego().step(dtSeconds);
        for (CarBlock npc : w.npcs()) {
            npc.step(dtSeconds);
        }

        for (PedestrianBlock p : w.pedestrians()) {
            p.step(dtSeconds);
        }
    }

    private void maybeSpawnNpc(WorldState w) {
        // Spawn rate with some randomness
        double spawnProb = 0.65;
        if (rng.nextDouble() > spawnProb) {
            spawnCooldownS = 0.4 + rng.nextDouble() * 0.8;
            return;
        }

        double npcW = 34;
        double npcH = 58;
        double y = npcSpawnY;

        // Try a few times to find a non-overlapping x position.
        int tries = 0;
        while (tries++ < 8) {
            double roadLeft = w.width() / 2.0 - 110.0;
            double roadRight = w.width() / 2.0 + 110.0;

            double x = laneXCenter - npcW / 2.0 + rng.nextGaussian() * 10.0;
            x = Math.max(roadLeft, Math.min(roadRight - npcW, x));

            CarBlock candidate = new CarBlock(
                    "npc-" + (++npcSeq),
                    CarBlock.Kind.NPC,
                    new Vec2(x, y),
                    new Vec2(0.0, 70 + rng.nextDouble() * 120), // px/s
                    npcW,
                    npcH
            );

            if (!overlapsAnyCarOrEgo(w, candidate)) {
                w.addNpc(candidate);
                spawnCooldownS = 0.5 + rng.nextDouble() * 1.2;
                return;
            }
        }

        // Couldn't find space; wait and try later.
        spawnCooldownS = 0.2 + rng.nextDouble() * 0.4;
    }

    private void maybeSpawnPedestrian(WorldState w) {
        double spawnProb = 0.45;
        if (rng.nextDouble() > spawnProb) {
            pedSpawnCooldownS = 0.3 + rng.nextDouble() * 0.8;
            return;
        }

        double pedW = 12;
        double pedH = 26;
        double y = npcSpawnY;

        int tries = 0;
        while (tries++ < 10) {
            double roadLeft = w.width() / 2.0 - 110.0;
            double roadRight = w.width() / 2.0 + 110.0;

            double x = roadLeft + rng.nextDouble() * (roadRight - roadLeft - pedW);
            Vec2 pos = new Vec2(x, y);

            // Simple "walking towards ego" drift.
            double speed = 40 + rng.nextDouble() * 80; // px/s
            Vec2 vel = new Vec2(0.0, speed);

            PedestrianBlock candidate = new PedestrianBlock(
                    "ped-" + (++pedSeq),
                    pos,
                    vel,
                    pedW,
                    pedH
            );

            if (!overlapsAnyCarOrPed(w, candidate)) {
                w.addPedestrian(candidate);
                pedSpawnCooldownS = 0.6 + rng.nextDouble() * 1.4;
                return;
            }
        }

        pedSpawnCooldownS = 0.2 + rng.nextDouble() * 0.4;
    }

    private static boolean overlapsAnyCarOrEgo(WorldState w, CarBlock candidate) {
        Aabb c = candidate.aabb();

        if (w.ego().aabb().intersects(c)) return true;
        for (CarBlock other : w.npcs()) {
            if (other.aabb().intersects(c)) return true;
        }
        return false;
    }

    private static boolean overlapsAnyCarOrPed(WorldState w, PedestrianBlock candidate) {
        Aabb c = candidate.aabb();

        if (w.ego().aabb().intersects(c)) return true;
        for (CarBlock other : w.npcs()) {
            if (other.aabb().intersects(c)) return true;
        }
        for (PedestrianBlock other : w.pedestrians()) {
            if (other.aabb().intersects(c)) return true;
        }
        return false;
    }
}

