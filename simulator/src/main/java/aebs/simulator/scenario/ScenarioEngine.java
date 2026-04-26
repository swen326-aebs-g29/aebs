package aebs.simulator.scenario;

import aebs.simulator.model.Aabb;
import aebs.simulator.model.Vec2;
import aebs.simulator.world.CarBlock;
import aebs.simulator.world.PedestrianBlock;
import aebs.simulator.world.WorldState;

import java.util.Random;
import java.util.Optional;

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

    // Keep density reasonable so avoidance has room to work.
    private static final int MAX_NPCS = 8;
    private static final int MAX_PEDS = 10;
    private static final double SPAWN_CLEARANCE_MARGIN_CAR_PX = 18.0;
    private static final double SPAWN_CLEARANCE_MARGIN_PED_PX = 12.0;

    public ScenarioEngine(long seed, double laneXCenter, double npcSpawnY) {
        this.rng = new Random(seed);
        this.laneXCenter = laneXCenter;
        this.npcSpawnY = npcSpawnY;
    }

    /** Resets spawn timers and sequences (RNG stream continues). */
    public void reset() {
        spawnCooldownS = 0.0;
        pedSpawnCooldownS = 0.0;
        npcSeq = 0;
        pedSeq = 0;
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

        // If there is a collision, stop ego immediately and skip overlap resolution.
        // (Overlap resolution would otherwise separate objects before the UI can register the impact.)
        Optional<aebs.simulator.world.Collidable> hit = w.firstCollisionWithEgo();
        if (hit.isPresent()) {
            w.ego().setCollided(true);
            hit.get().setCollided(true);
            w.setBrakeCommand(1.0);
            w.ego().setVel(new Vec2(0.0, 0.0));
            return;
        }

        // Ensure objects do not "layer" on top of each other after movement.
        resolveOverlaps(w);
    }

    private void resolveOverlaps(WorldState w) {
        double roadLeft = w.width() / 2.0 - 110.0;
        double roadRight = w.width() / 2.0 + 110.0;

        // Cars vs cars: if any overlap occurs, nudge later-spawned cars sideways.
        CarBlock[] cars = w.npcs().toArray(new CarBlock[0]);
        for (int i = 0; i < cars.length; i++) {
            for (int j = i + 1; j < cars.length; j++) {
                CarBlock a = cars[i];
                CarBlock b = cars[j];
                if (!a.aabb().intersects(b.aabb())) continue;

                // Push b away from a, left or right depending on relative center.
                double dir = (b.aabb().centerX() >= a.aabb().centerX()) ? 1.0 : -1.0;
                double push = 18.0 * dir;
                double newX = clamp(b.pos().x() + push, roadLeft, roadRight - b.aabb().w());
                b.setVel(new Vec2(b.vel().x(), b.vel().y())); // keep vel object valid
                // update position by directly stepping a tiny amount in x via velocity
                // (no direct setPos API), so we approximate by setting a one-tick lateral velocity
                b.setVel(new Vec2(push / 0.02, b.vel().y()));
                b.step(0.02);
                b.setVel(new Vec2(0.0, b.vel().y()));
            }
        }

        // Pedestrians vs pedestrians: nudge sideways similarly.
        PedestrianBlock[] peds = w.pedestrians().toArray(new PedestrianBlock[0]);
        for (int i = 0; i < peds.length; i++) {
            for (int j = i + 1; j < peds.length; j++) {
                PedestrianBlock a = peds[i];
                PedestrianBlock b = peds[j];
                if (!a.aabb().intersects(b.aabb())) continue;

                double dir = (b.aabb().centerX() >= a.aabb().centerX()) ? 1.0 : -1.0;
                double push = 10.0 * dir;
                double newX = clamp(b.pos().x() + push, roadLeft, roadRight - b.aabb().w());
                b.setVel(new Vec2(push / 0.02, b.vel().y()));
                b.step(0.02);
                b.setVel(new Vec2(0.0, b.vel().y()));
            }
        }

        // Cars vs pedestrians: prevent intersections between the two groups.
        for (CarBlock car : cars) {
            for (PedestrianBlock ped : peds) {
                if (!car.aabb().intersects(ped.aabb())) continue;

                // Prefer nudging the pedestrian slightly up/down off the car footprint.
                Vec2 pedVelSaved = ped.vel();
                double dirY = (ped.aabb().centerY() >= car.aabb().centerY()) ? 1.0 : -1.0;
                double pushY = 14.0 * dirY;
                ped.setVel(new Vec2(pedVelSaved.x(), pushY / 0.02));
                ped.step(0.02);
                ped.setVel(pedVelSaved);

                // If still intersecting (rare), nudge the car sideways a bit.
                if (car.aabb().intersects(ped.aabb())) {
                    double dirX = (car.aabb().centerX() >= ped.aabb().centerX()) ? 1.0 : -1.0;
                    double pushX = 16.0 * dirX;
                    double newX = clamp(car.pos().x() + pushX, roadLeft, roadRight - car.aabb().w());
                    car.setVel(new Vec2((newX - car.pos().x()) / 0.02, car.vel().y()));
                    car.step(0.02);
                    car.setVel(new Vec2(0.0, car.vel().y()));
                }
            }
        }

        resolveEgoVsTraffic(w);
    }

    /**
     * Keep the ego from ending a tick overlapping NPCs or pedestrians (radar/camera can miss;
     * lateral avoidance may not complete in one frame).
     */
    private void resolveEgoVsTraffic(WorldState w) {
        double roadLeft = w.width() / 2.0 - 110.0;
        double roadRight = w.width() / 2.0 + 110.0;
        CarBlock ego = w.ego();
        // Keep a small visual/physical buffer so ego isn't "door-to-door" after a pass.
        final double margin = 14.0;

        for (int iter = 0; iter < 8; iter++) {
            boolean any = false;
            for (CarBlock npc : w.npcs()) {
                any |= separateEgoFromObstacle(ego, npc.aabb(), roadLeft, roadRight, margin);
            }
            for (PedestrianBlock ped : w.pedestrians()) {
                any |= separateEgoFromObstacle(ego, ped.aabb(), roadLeft, roadRight, margin);
            }
            if (!any) break;
        }
    }

    private static boolean separateEgoFromObstacle(
            CarBlock ego, Aabb obs, double roadLeft, double roadRight, double margin
    ) {
        Aabb e = ego.aabb();
        if (!e.intersects(obs)) return false;

        double egoW = e.w();
        double x = ego.pos().x();

        double leftSlot = clamp(obs.x() - margin - egoW, roadLeft, roadRight - egoW);
        double rightSlot = clamp(obs.x() + obs.w() + margin, roadLeft, roadRight - egoW);

        double dLeft = Math.abs(x - leftSlot);
        double dRight = Math.abs(x - rightSlot);
        double nx = dLeft <= dRight ? leftSlot : rightSlot;
        double dx = nx - x;
        ego.translate(dx, 0.0);
        return true;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void maybeSpawnNpc(WorldState w) {
        if (w.npcs().size() >= MAX_NPCS) {
            spawnCooldownS = 0.9 + rng.nextDouble() * 1.4;
            return;
        }
        // Spawn rate with some randomness (higher = more traffic / obstacles).
        double spawnProb = 0.28;
        if (rng.nextDouble() > spawnProb) {
            spawnCooldownS = 1.2 + rng.nextDouble() * 1.8;
            return;
        }

        double npcW = 34;
        double npcH = 58;
        double y = npcSpawnY;

        // Try a few times to find a non-overlapping x position.
        int tries = 0;
        while (tries++ < 12) {
            double roadLeft = w.width() / 2.0 - 110.0;
            double roadRight = w.width() / 2.0 + 110.0;

            double x = laneXCenter - npcW / 2.0 + rng.nextGaussian() * 10.0;
            x = Math.max(roadLeft, Math.min(roadRight - npcW, x));

            CarBlock candidate = new CarBlock(
                    "npc-" + (++npcSeq),
                    CarBlock.Kind.NPC,
                    new Vec2(x, y),
                    // Slower traffic so the simulation is easier to follow.
                    new Vec2(0.0, 18 + rng.nextDouble() * 36), // px/s
                    npcW,
                    npcH
            );

            if (!overlapsAnyCarOrEgo(w, candidate, SPAWN_CLEARANCE_MARGIN_CAR_PX)) {
                w.addNpc(candidate);
                spawnCooldownS = 1.4 + rng.nextDouble() * 2.2;
                return;
            }
        }

        // Couldn't find space; wait and try later.
        spawnCooldownS = 0.8 + rng.nextDouble() * 1.1;
    }

    private void maybeSpawnPedestrian(WorldState w) {
        if (w.pedestrians().size() >= MAX_PEDS) {
            pedSpawnCooldownS = 0.9 + rng.nextDouble() * 1.4;
            return;
        }
        double spawnProb = 0.26;
        if (rng.nextDouble() > spawnProb) {
            pedSpawnCooldownS = 1.2 + rng.nextDouble() * 1.8;
            return;
        }

        double pedW = 12;
        double pedH = 26;

        int tries = 0;
        while (tries++ < 14) {
            double roadLeft = w.width() / 2.0 - 110.0;
            double roadRight = w.width() / 2.0 + 110.0;

            double yPos = npcSpawnY;

            double x = laneXCenter - pedW / 2.0 + rng.nextGaussian() * 12.0;
            x = Math.max(roadLeft, Math.min(roadRight - pedW, x));
            Vec2 pos = new Vec2(x, yPos);

            // Down the road like NPC traffic, with a little lateral drift.
            double vy = 14 + rng.nextDouble() * 32;
            double vx = rng.nextGaussian() * 10.0;
            Vec2 vel = new Vec2(vx, vy);

            PedestrianBlock candidate = new PedestrianBlock(
                    "ped-" + (++pedSeq),
                    pos,
                    vel,
                    pedW,
                    pedH
            );

            if (!overlapsAnyCarOrPed(w, candidate, SPAWN_CLEARANCE_MARGIN_PED_PX)) {
                w.addPedestrian(candidate);
                pedSpawnCooldownS = 1.4 + rng.nextDouble() * 2.2;
                return;
            }
        }

        pedSpawnCooldownS = 0.8 + rng.nextDouble() * 1.1;
    }

    private static boolean overlapsAnyCarOrEgo(WorldState w, CarBlock candidate, double marginPx) {
        Aabb c = candidate.aabb();

        if (intersectsInflated(w.ego().aabb(), c, marginPx)) return true;
        for (CarBlock other : w.npcs()) {
            if (intersectsInflated(other.aabb(), c, marginPx)) return true;
        }
        return false;
    }

    private static boolean overlapsAnyCarOrPed(WorldState w, PedestrianBlock candidate, double marginPx) {
        Aabb c = candidate.aabb();

        if (intersectsInflated(w.ego().aabb(), c, marginPx)) return true;
        for (CarBlock other : w.npcs()) {
            if (intersectsInflated(other.aabb(), c, marginPx)) return true;
        }
        for (PedestrianBlock other : w.pedestrians()) {
            if (intersectsInflated(other.aabb(), c, marginPx)) return true;
        }
        return false;
    }

    private static boolean intersectsInflated(Aabb a, Aabb b, double m) {
        Aabb ai = new Aabb(a.x() - m, a.y() - m, a.w() + 2.0 * m, a.h() + 2.0 * m);
        Aabb bi = new Aabb(b.x() - m, b.y() - m, b.w() + 2.0 * m, b.h() + 2.0 * m);
        return ai.intersects(bi);
    }
}

