#ifndef LineFollower_h
#define LineFollower_h

class LineFollower {
  public:
    LineFollower();
    void test();
    uint8_t get_line_reading(int);
    int16_t get_distance(uint8_t);
    
    void calibrate();
    void calibrate_show();
    void calibrate_reset();
    
  private:
    int cal_min[8];
    int cal_max[8];
};

#endif