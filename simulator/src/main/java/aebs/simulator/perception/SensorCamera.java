package aebs.simulator.perception;

public interface SensorCamera {
    String classification(); // classification of objects
    Long timestamp();      // update time (ms since epoch)
    Double confidence();   // confidence [0..1]
}
