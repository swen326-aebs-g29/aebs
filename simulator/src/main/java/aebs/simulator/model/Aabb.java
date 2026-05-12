package aebs.simulator.model;

public record Aabb(double x, double y, double w, double h) {
    public boolean intersects(Aabb o) {
        return x < o.x + o.w && x + w > o.x && y < o.y + o.h && y + h > o.y;
    }

    public double centerX() { return x + w / 2.0; }
    public double centerY() { return y + h / 2.0; }
}

