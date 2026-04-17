package aebs.simulator.world;

import aebs.simulator.model.Aabb;
import aebs.simulator.model.Vec2;

public final class CarBlock implements Collidable {
    public enum Kind { EGO, NPC }

    private final String id;
    private final Kind kind;
    private Vec2 pos;
    private Vec2 vel;
    private final double w;
    private final double h;
    private boolean collided;

    public CarBlock(String id, Kind kind, Vec2 pos, Vec2 vel, double w, double h) {
        this.id = id;
        this.kind = kind;
        this.pos = pos;
        this.vel = vel;
        this.w = w;
        this.h = h;
    }

    public String id() { return id; }
    public Kind kind() { return kind; }
    public Vec2 pos() { return pos; }
    public Vec2 vel() { return vel; }
    public void setVel(Vec2 v) { this.vel = v; }
    public boolean collided() { return collided; }
    public void setCollided(boolean v) { this.collided = v; }

    public Aabb aabb() { return new Aabb(pos.x(), pos.y(), w, h); }

    public void step(double dtSeconds) {
        pos = pos.add(vel.mul(dtSeconds));
    }
}

