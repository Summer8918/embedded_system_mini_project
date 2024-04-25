# embedded_system_mini_project
Group members: Jie Yang, Sanjay Gounder, Adam Blakeslee

## Project Overview
We built a simple command-line interface for some basic tasks using an RTOS to interact with hardware.
![screenshot](./pictures/overview.png)

* The microcontroller that has FreeRTOS is connected to a PC via UART.
* The UART Parse thread process user commands and push the command to Command Queue.
* The Command Queue is used to store commands from user.
* The Router thread distributes tasks for worker threads.
* There are two worker threads, LED and Motor threads, each thread works on separate task.
* The function procedure is shown below: 
1. PC sends message via UART, and the UART interrupt handler will save user's input.
2. When a command reaches the end (terminated with ‘\n’), the UART interrupt handler will wake up the Uart Parse thread.
3. Then the Uart Parse thread starts processing the command, and pushes it to the command queue.
4. Every time the Uart Parse thread pushes a new command to the command queue, it will wake up the router thread to check if the corresponding worker thread is idle.
5. When the worker thread (currently LED and Motor thread) is idle, the router thread will fetch the command for that worker thread, assign the task to the worker thread, and then wake up the worker thread.
6. When a worker thread finishes the task, it will go to sleep and wake up the router thread to schedule tasks.


![screenshot](./pictures/systemcommands.png)
