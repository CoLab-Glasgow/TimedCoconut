enum DrawerStateCode {
    DRAWER_CLOSED = 0,
    DRAWER_OPEN = 1
};

class Drawer {
public:
    Drawer() : hour(0), minutes(0), state(DRAWER_CLOSED) {}

    Drawer(int h, int m)
        : hour(h), minutes(m), state(DRAWER_CLOSED) {}

    __attribute__((noinline, used)) int get_the_hour() const {
        return hour;
    }

    __attribute__((noinline, used)) int get_minutes() const {
        return minutes;
    }

    __attribute__((noinline, used)) void SetDrawerStateCode(int s) {
        state = s;
    }

    __attribute__((noinline, used)) int GetDrawerStateCode() const {
        return state;
    }

     int hour;
    int minutes;
    volatile int state;
private:
   
};