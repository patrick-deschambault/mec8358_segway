#ifndef PERIODIC_TASK_H
#define PERIODIC_TASK_H

class PeriodicTask {
private:
    unsigned int previousMillis;
    unsigned int interval;
    void (*task)(); // Utilisation d'un pointeur de fonction simple

public:
    PeriodicTask(unsigned int interval, void (*task)()); // Constructeur
    void update();
};

#endif // PERIODIC_TASK_H
