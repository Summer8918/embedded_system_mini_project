#include <semaphore>
#include <thread>
#include <iostream>

using namespace std;

// Initialize a semaphore with an initial count of 1
// only one thread can access the resource protected by sem at a time.
std::counting_semaphore<10> countSem(1);
std::binary_semaphore ledRouterSem{0};
std::binary_semaphore motorRouterSem{0};

std::mutex led_status_mutex;
bool led_busy = false;
std::mutex motor_status_mutex;
bool motor_busy = false;

std::mutex queue_motor_item_num_mutex;
std::mutex queue_led_item_num_mutex;
int queue_motor_item_num = 0;
int queue_led_item_num = 0;

void ThreadRouter() {
    while (1) {
        countSem.acquire();
        std::cout << "[Router thread] Got the signal\n"; // response message

        queue_led_item_num_mutex.lock();
        int tmp_item_num = queue_led_item_num;
        queue_led_item_num_mutex.unlock();

        cout << "Router: 0 queue_led_item_num:" << tmp_item_num << endl;
        if (tmp_item_num > 0) {
            
            led_status_mutex.lock();
            
            if (led_busy == false) {
                led_status_mutex.unlock();
                std::cout << "wake up led worker\n"; // message
                ledRouterSem.release();
                queue_led_item_num_mutex.lock();
                queue_led_item_num--;
                queue_led_item_num_mutex.unlock();
            } else {
                led_status_mutex.unlock();
            }
        }

        queue_motor_item_num_mutex.lock();
        tmp_item_num = queue_motor_item_num;
        queue_motor_item_num_mutex.unlock();

        cout << "Router: 1 queue_motor_item_num:" << tmp_item_num << endl;
        if (tmp_item_num > 0) {
            motor_status_mutex.lock();
            if (motor_busy == false) {
                motor_status_mutex.unlock();
                std::cout << "wake up motor worker\n";
                motorRouterSem.release();
                queue_motor_item_num_mutex.lock();
                queue_motor_item_num--;
                queue_motor_item_num_mutex.unlock();
            } else {
                motor_status_mutex.unlock();
            }
        }

        std::cout << "[Router thread] go to sleep\n"; // message
    }
}

void ThreadMotor() {
    while (1) {
        motorRouterSem.acquire();
        motor_status_mutex.lock();
        motor_busy = true;
        motor_status_mutex.unlock();
        std::cout << "[Motor thread] Got the signal\n"; // response message
 
        using namespace std::literals;
        std::this_thread::sleep_for(1s);

        std::cout << "[Motor thread] go to sleep\n"; // message
        motor_status_mutex.lock();
        motor_busy = false;
        motor_status_mutex.unlock();
        countSem.release();
    }
}

void ThreadLed() {
    while(1) {
        ledRouterSem.acquire();
        led_status_mutex.lock();
        led_busy = true;
        led_status_mutex.unlock();
        std::cout << "[LED thread] Got the signal\n"; // response message

        using namespace std::literals;
        std::this_thread::sleep_for(1s);
        std::cout << "[led thread] go to sleep\n"; // message
        led_status_mutex.lock();
        led_busy = false;
        led_status_mutex.unlock();
        countSem.release();
    }
}

void ThreadUart() {
    std::cout << "Uart thread running\n"; // response message
 
    // wait for 3 seconds to imitate some work
    // being done by the thread
    using namespace std::literals;
    queue_led_item_num = 10;
    queue_motor_item_num = 10;
    int tmp_queue_item_num = 0;
    for (int i = 1; i < 10; i++) {
        queue_led_item_num_mutex.lock();
        queue_led_item_num += i;
        tmp_queue_item_num = queue_led_item_num;
        queue_led_item_num_mutex.unlock();
        std::cout << "[uart thread]queue_led_item_num" << tmp_queue_item_num << "\n"; // message

        queue_motor_item_num_mutex.lock();
        queue_motor_item_num += i;
        tmp_queue_item_num = queue_motor_item_num;
        queue_motor_item_num_mutex.unlock();
        std::cout << "[uart thread]queue_motor_item_num" << tmp_queue_item_num << "\n"; // message

        countSem.release();
        std::this_thread::sleep_for(10s);
    }
}


int main() {
    std::thread ledWorker(ThreadLed);
    std::thread uartParser(ThreadUart);
    std::thread motorWorker(ThreadMotor);
    std::thread router(ThreadRouter);
    ledWorker.join();
    uartParser.join();
    motorWorker.join();
    router.join();
    return 0;
}

