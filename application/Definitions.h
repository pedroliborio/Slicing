enum WavePsid {
    Entertainment_A = 40,
    Entertainment_B = 41
};

enum WaveEntServiceState{
    REQUESTING = 0,
    RECEIVING = 1
};

//XXX Network Metrics Statistics
struct t_NetMetrics{
    simtime_t delaySum = SimTime(0);
    simtime_t jitterSum = SimTime(0);
    simtime_t lastDelay = SimTime(0);
    simtime_t timeRxFirst = SimTime(0);
    simtime_t timeRxLast = SimTime(0);
    simtime_t timeTxFirst = SimTime(0);
    simtime_t timeTxLast = SimTime(0);
    uint32_t txBytesSum = 0;
    uint32_t rxBytesSum = 0;
    uint32_t generatedPackets = 0;
    uint32_t receivedPackets = 0;
};
typedef struct t_NetMetrics NetMetrics;

