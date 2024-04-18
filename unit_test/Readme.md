semaphore_counter.cpp simulate the synchronization between uart, router, led and motor threads.

To test semaphore_counter.cpp, run the following command in PC to compile:
g++ semaphore_counter.cpp -std=c++20 -o sem
Then run command:
./sem > res.txt

The stdout is stored in file res.txt
Currently, led and motor thread will be woken up 55 times, it works as expected.
