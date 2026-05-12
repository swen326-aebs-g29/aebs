package aebs.simulator.perception;

public interface SensorWheelSpeed {
    Double wheelSpeed(int wheelIndex); // vehicle speed derived per wheel
    Double RPM();                      // revolutions per minute
    Long timestamp();                  // update time (ms since epoch)
}
