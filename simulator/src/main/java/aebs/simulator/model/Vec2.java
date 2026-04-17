package aebs.simulator.model;

public record Vec2(double x, double y) {
    public Vec2 add(Vec2 o) { return new Vec2(x + o.x, y + o.y); }
    public Vec2 mul(double k) { return new Vec2(x * k, y * k); }
}

