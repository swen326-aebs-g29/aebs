package aebs.simulator.world;

import aebs.simulator.model.Aabb;
import aebs.simulator.model.Vec2;

public final class PedestrianBlock implements Collidable {
    private final String id;
    private Vec2 pos;
    private Vec2 vel;
    private final double w;
    private final double h;
    private boolean collided;

    public PedestrianBlock(String id, Vec2 pos, Vec2 vel, double w, double h) {
        this.id = id;
        this.pos = pos;
        this.vel = vel;
        this.w = w;
        this.h = h;
    }

    public String id() { return id; }
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

