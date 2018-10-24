```
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
```

## Sections to split up
- balance controller (section 2) - Phoneix
- odometry and motion control (section 3) - Johnson (will provide limited help for other sections)
- path planning and following (section 4) -

## How to automatically sync code from computer to Beaglebone:
1. (on computer) ssh-keygen to default `id_rsa` and `id_rsa.pub`
2. (on bbg) copy content of computer's `id_rsa.pub` to bbg's `~/.ssh/authorized_keys` (create file/directory if necessary)
3. (on computer) `python -m --user pip install watchdog`
4. download script to computer: https://gist.github.com/LemonPi/49e749489a46186665ea4a5e5e0819c1
5. run script passing in path to directory to sync `python watch_sync.py ubalance/`

## Determining motor parameters
- in steady state V = Ri + Kw
    - start measuring after spinning for 2s to ensure steady state
    - measure V across motors
    - measure i using current sense
    - measure w using encoder
    - get multiple data points and estimate R and K using linear regression
        - V = [i w][R K]'
        - [V; ...; V_N] = [i_1 w_1; ...; i_N w_N][R K]'
            - V = Ix
        - [R K]' = pseudoInv(I)V
- steady state no load speed ω_NL = KV/(K^2 + Rb)
    - estimate b = (KV/(ω_NL) - K^2) / R
    - ignore c for modelling
- determine stall torque by holding onto wheel to stall it
    - T_s = KV/R
    - measure voltage across motors
