# embedded_system_mini_project
Group members: Jie Yang, Sanjay Gounder, Adam Blakeslee

## Project Overview
We build a simple command-line interface for some basic tasks based on RTOS to interact with hardware.
![screenshot](./pictures/overview.png)

* The microcontroller that has an RTOS is connected to a PC via UART.
* The UART Parse thread process user commands and push the command to Command Queue.
* The Command Queue is used to store commands from user.
* The Router thread distributes tasks for worker threads.
* There are two worker threads, LED and Motor threads, each thread works on separate task.
* The function procedure is shown below: 
1. PC sends message via UART, and the UART interrupt handler will save user's input.
2. When a command reaches the end (like end with ‘\n’), the UART interrupt handler will wake up the Uart Parse thread.
3. Then the Uart Parse thread starts processing the command, and push it to the command queue.
4. Every time when the Uart Parse thread pushes a new command to the command queue, it will wake up the router thread so the router thread will check if the LED and Motor thread is idle.
5. When the certain worker thread (currently LED and Motor thread) is idle, the router thread will fetch a command that the worker thread processes, and assign the task to the worker thread, and then wake up the worker thread.
6. When a worker thread finishes the task, it will go to sleep and wakes up the router thread to schedule tasks.
