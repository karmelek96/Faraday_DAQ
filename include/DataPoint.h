union DataPoint {
  struct {
    uint32_t time;
    uint8_t state;
    uint8_t flags;
    double thrust;
    double pressure;
    String toString() {
      String s = "";
      s += time;
      s += ",";
      s += state;
      s += ",";
      s += flags;
      s += ",";
      s += thrust;
      s += ",";
      s += pressure;
      return s;
    }
  } data;
  uint8_t bytes[sizeof(data)];
};

union DataPointCastrated {
  struct {
    uint32_t time;
    uint8_t state;
    uint8_t flags;
    uint16_t thrust;
    uint8_t pressure;
  } data;
  uint8_t bytes[sizeof(data)];
};