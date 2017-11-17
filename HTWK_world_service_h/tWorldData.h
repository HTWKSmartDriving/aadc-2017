
class tWorldData {
public:
    void *data;
    cReadWriteMutex *lock;
    size_t size;
};
