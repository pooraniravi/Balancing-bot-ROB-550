/*******************************************************************************
*                           Balancebot Template Code
*                           pgaskell@umich.edu
*       
*******************************************************************************/

bin/			             : Binaries folder
balanceebot/balanceebot.c/.h : Main setup and threads
test_motors/test_motors.c/.h : Program to test motor implementation
common/mb_controller.c/.h    : Contoller for manual and autonomous nav
common/mb_defs.h             : Define hardware config
common/mb_motors.c/.h        : Motor functions to be used by balancebot
common/mb_odometry.c/.h	     : Odometry functions

How to automatically sync code from computer to Beaglebone:
1. (on computer) ssh-keygen to default `id_rsa` and `id_rsa.pub`
2. (on bbg) copy content of computer's `id_rsa.pub` to bbg's `~/.ssh/authorized_keys` (create file/directory if necessary)
3. (on computer) `python -m --user pip install watchdog`
4. download script to computer: https://gist.github.com/LemonPi/49e749489a46186665ea4a5e5e0819c1
5. run script passing in path to directory to sync `python watch_sync.py ubalance/`