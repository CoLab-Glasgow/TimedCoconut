enum RedLedState {
    REDLED_OFF = 0,
    REDLED_ON = 1,
    REDLED_BLINKING = 2
};

class RedLed {
public:
    RedLed() : state(REDLED_OFF) {}

    __attribute__((noinline, used)) void setRedLedState(int s) {
        state = s;
    }

    __attribute__((noinline, used)) int getRedLedState() const {
        return state;
    }

private:
    volatile int state;
};