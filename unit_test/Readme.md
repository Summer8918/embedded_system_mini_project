simulate_sync.cpp simulate the synchronization between uart, router, led and motor threads.

To test simulate_sync.cpp, run the following command in PC to compile:
g++ simulate_sync.cpp -std=c++20 -o sync
Then run command:
./sync > res.txt

The stdout is stored in file res.txt
Currently, led and motor thread will be woken up 55 times, it works as expected.
