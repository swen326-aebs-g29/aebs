package aebs.simulator.perception;

public interface SensorRadar {
    double distance();      // distance between vehicle and object
    long timestamp();       // update time (ms since epoch)
    String speedObject();   // relative speed of an object (human-readable)
}
