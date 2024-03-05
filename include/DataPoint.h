union DataPoint {
  struct {
    uint32_t time;
    uint8_t state;
    uint8_t flags;
    double thrust;
    double pressure;
    double temp1;
    double temp2;
    double temp3;
    double temp4;
  } data;
  uint8_t bytes[sizeof(data)];
};