package aebs.simulator.world;

import aebs.simulator.model.Aabb;
import aebs.simulator.model.Vec2;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

public final class WorldState {
    private final double width;
    private final double height;
    private final CarBlock ego;
    private final List<CarBlock> npcs = new ArrayList<>();
    private final List<PedestrianBlock> pedestrians = new ArrayList<>();

    public WorldState(double width, double height, CarBlock ego) {
        this.width = width;
        this.height = height;
        this.ego = ego;
    }

    public double width() { return width; }
    public double height() { return height; }
    public CarBlock ego() { return ego; }
    public List<CarBlock> npcs() { return Collections.unmodifiableList(npcs); }
    public List<PedestrianBlock> pedestrians() { return Collections.unmodifiableList(pedestrians); }

    public void addNpc(CarBlock npc) { npcs.add(npc); }
    public void removeNpc(CarBlock npc) { npcs.remove(npc); }

    public void addPedestrian(PedestrianBlock p) { pedestrians.add(p); }
    public void removePedestrian(PedestrianBlock p) { pedestrians.remove(p); }

    public List<CarBlock> allCars() {
        List<CarBlock> all = new ArrayList<>(1 + npcs.size());
        all.add(ego);
        all.addAll(npcs);
        return all;
    }

    public Optional<Collidable> firstCollisionWithEgo() {
        Aabb e = ego.aabb();
        for (CarBlock c : npcs) {
            if (e.intersects(c.aabb())) return Optional.of(c);
        }
        for (PedestrianBlock p : pedestrians) {
            if (e.intersects(p.aabb())) return Optional.of(p);
        }
        return Optional.empty();
    }

    public double egoSpeedPixelsPerSec() {
        Vec2 v = ego.vel();
        return Math.hypot(v.x(), v.y());
    }
}

